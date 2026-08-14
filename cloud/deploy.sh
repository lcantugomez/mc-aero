#!/usr/bin/env bash
# Provision the MC Aero telemetry sink (S3 + IAM role + Lambda + Function URL).
# Idempotent-ish: safe to re-run; existing resources are reused/updated.
set -euo pipefail
export AWS_PAGER=""

PROFILE="${AWS_PROFILE:-luis.cantugomez}"
REGION="${AWS_REGION:-us-east-2}"
ACCOUNT="$(aws sts get-caller-identity --profile "$PROFILE" --query Account --output text)"

BUCKET="${BUCKET:-mc-aero-telemetry-${ACCOUNT}-${REGION}}"
FUNC="${FUNC:-mc-aero-telemetry-sink}"
ROLE="${ROLE:-mc-aero-telemetry-sink-role}"
PREFIX="${PREFIX:-telemetry}"
SECRET_FILE="${SECRET_FILE:-$HOME/.mc-aero-telemetry-secret}"

HERE="$(cd "$(dirname "$0")" && pwd)"
AWS=(aws --profile "$PROFILE" --region "$REGION" --no-cli-pager)

echo ">> account=$ACCOUNT region=$REGION"
echo ">> bucket=$BUCKET func=$FUNC role=$ROLE"

# --- shared secret (create once, reuse thereafter) -------------------------
if [ -f "$SECRET_FILE" ]; then
    SECRET="$(cat "$SECRET_FILE")"
    echo ">> reusing existing secret from $SECRET_FILE"
else
    SECRET="$(openssl rand -hex 24)"
    ( umask 177; printf '%s' "$SECRET" > "$SECRET_FILE" )
    echo ">> generated new secret -> $SECRET_FILE"
fi

# --- S3 bucket -------------------------------------------------------------
if "${AWS[@]}" s3api head-bucket --bucket "$BUCKET" 2>/dev/null; then
    echo ">> bucket exists"
else
    echo ">> creating bucket"
    "${AWS[@]}" s3api create-bucket --bucket "$BUCKET" \
        --create-bucket-configuration "LocationConstraint=$REGION"
fi
"${AWS[@]}" s3api put-public-access-block --bucket "$BUCKET" \
    --public-access-block-configuration \
    BlockPublicAcls=true,IgnorePublicAcls=true,BlockPublicPolicy=true,RestrictPublicBuckets=true

# --- IAM role --------------------------------------------------------------
TRUST='{"Version":"2012-10-17","Statement":[{"Effect":"Allow","Principal":{"Service":"lambda.amazonaws.com"},"Action":"sts:AssumeRole"}]}'
S3DOC="{\"Version\":\"2012-10-17\",\"Statement\":[{\"Sid\":\"WriteTelemetry\",\"Effect\":\"Allow\",\"Action\":\"s3:PutObject\",\"Resource\":\"arn:aws:s3:::${BUCKET}/${PREFIX}/*\"}]}"

if aws --profile "$PROFILE" iam get-role --role-name "$ROLE" >/dev/null 2>&1; then
    echo ">> role exists"
else
    echo ">> creating role"
    aws --profile "$PROFILE" iam create-role --role-name "$ROLE" \
        --assume-role-policy-document "$TRUST" >/dev/null
fi
aws --profile "$PROFILE" iam attach-role-policy --role-name "$ROLE" \
    --policy-arn arn:aws:iam::aws:policy/service-role/AWSLambdaBasicExecutionRole
aws --profile "$PROFILE" iam put-role-policy --role-name "$ROLE" \
    --policy-name write-telemetry --policy-document "$S3DOC"
ROLE_ARN="$(aws --profile "$PROFILE" iam get-role --role-name "$ROLE" --query Role.Arn --output text)"
echo ">> role arn=$ROLE_ARN"

# --- package Lambda --------------------------------------------------------
ZIP="/tmp/mc-aero-func.zip"
rm -f "$ZIP"
# Build the deployment package with Python's stdlib (no `zip` binary needed).
( cd "$HERE" && python3 -m zipfile -c "$ZIP" telemetry_lambda.py )

ENV="Variables={BUCKET=$BUCKET,API_KEY=$SECRET,PREFIX=$PREFIX}"

if "${AWS[@]}" lambda get-function --function-name "$FUNC" >/dev/null 2>&1; then
    echo ">> updating function code + config"
    "${AWS[@]}" lambda update-function-code --function-name "$FUNC" \
        --zip-file "fileb://$ZIP" >/dev/null
    "${AWS[@]}" lambda wait function-updated --function-name "$FUNC"
    "${AWS[@]}" lambda update-function-configuration --function-name "$FUNC" \
        --environment "$ENV" --timeout 15 --memory-size 128 >/dev/null
else
    echo ">> creating function (retrying while IAM role propagates)"
    for attempt in 1 2 3 4 5 6; do
        if "${AWS[@]}" lambda create-function --function-name "$FUNC" \
            --runtime python3.13 --role "$ROLE_ARN" \
            --handler telemetry_lambda.handler --zip-file "fileb://$ZIP" \
            --timeout 15 --memory-size 128 --environment "$ENV" >/dev/null 2>/tmp/lambda_err; then
            break
        fi
        echo "   attempt $attempt failed, waiting for role propagation..."
        sleep 10
        if [ "$attempt" = 6 ]; then echo "create-function failed:"; cat /tmp/lambda_err; exit 1; fi
    done
fi
"${AWS[@]}" lambda wait function-active --function-name "$FUNC"

# cap blast radius of a public URL
"${AWS[@]}" lambda put-function-concurrency --function-name "$FUNC" \
    --reserved-concurrent-executions 2 >/dev/null

# --- Function URL (public, gated by x-api-key inside the function) ---------
if ! "${AWS[@]}" lambda get-function-url-config --function-name "$FUNC" >/dev/null 2>&1; then
    echo ">> creating function URL"
    "${AWS[@]}" lambda create-function-url-config --function-name "$FUNC" \
        --auth-type NONE >/dev/null
fi
# Public access needs BOTH of these statements: InvokeFunctionUrl carries the
# auth-type condition, and InvokeFunction must be granted separately (AWS
# rejects the auth-type condition on the InvokeFunction action).
"${AWS[@]}" lambda add-permission --function-name "$FUNC" \
    --statement-id FunctionURLAllowPublicAccess \
    --action lambda:InvokeFunctionUrl --principal '*' \
    --function-url-auth-type NONE >/dev/null 2>&1 || true
"${AWS[@]}" lambda add-permission --function-name "$FUNC" \
    --statement-id FunctionURLPublicInvokeFn \
    --action lambda:InvokeFunction --principal '*' >/dev/null 2>&1 || true

URL="$("${AWS[@]}" lambda get-function-url-config --function-name "$FUNC" --query FunctionUrl --output text)"

echo
echo "=================================================================="
echo "Endpoint : $URL"
echo "Secret   : stored in $SECRET_FILE (chmod 600)"
echo "Bucket   : s3://$BUCKET/$PREFIX/"
echo "=================================================================="
printf '%s\n' "$URL" > /tmp/mc-aero-endpoint.txt
