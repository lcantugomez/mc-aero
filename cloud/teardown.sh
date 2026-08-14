#!/usr/bin/env bash
# Remove everything deploy.sh created. Empties and deletes the bucket too.
set -uo pipefail
export AWS_PAGER=""

PROFILE="${AWS_PROFILE:-luis.cantugomez}"
REGION="${AWS_REGION:-us-east-2}"
ACCOUNT="$(aws sts get-caller-identity --profile "$PROFILE" --query Account --output text)"

BUCKET="${BUCKET:-mc-aero-telemetry-${ACCOUNT}-${REGION}}"
FUNC="${FUNC:-mc-aero-telemetry-sink}"
ROLE="${ROLE:-mc-aero-telemetry-sink-role}"
AWS=(aws --profile "$PROFILE" --region "$REGION" --no-cli-pager)

echo ">> deleting function $FUNC"
"${AWS[@]}" lambda delete-function --function-name "$FUNC" 2>/dev/null || true

echo ">> detaching role policies $ROLE"
aws --profile "$PROFILE" iam delete-role-policy --role-name "$ROLE" --policy-name write-telemetry 2>/dev/null || true
aws --profile "$PROFILE" iam detach-role-policy --role-name "$ROLE" \
    --policy-arn arn:aws:iam::aws:policy/service-role/AWSLambdaBasicExecutionRole 2>/dev/null || true
aws --profile "$PROFILE" iam delete-role --role-name "$ROLE" 2>/dev/null || true

echo ">> emptying + deleting bucket $BUCKET"
"${AWS[@]}" s3 rm "s3://$BUCKET" --recursive 2>/dev/null || true
"${AWS[@]}" s3api delete-bucket --bucket "$BUCKET" 2>/dev/null || true

echo ">> done"
