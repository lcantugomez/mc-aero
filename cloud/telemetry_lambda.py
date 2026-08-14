"""MC Aero telemetry sink (AWS Lambda, Python).

Deploy behind a Lambda Function URL (auth type: NONE). Access is gated by a
shared secret sent in the `x-api-key` header, compared in constant time.
Each POST body is a JSON array (or single object) of telemetry snapshots;
records are written to S3 as one NDJSON object per request.

Environment variables:
  BUCKET   (required)  destination S3 bucket name
  API_KEY  (required)  shared secret expected in the x-api-key header
  PREFIX   (optional)  key prefix, default "telemetry"

Handler: telemetry_lambda.handler
"""

import base64
import hmac
import json
import os
import time
import uuid

import boto3

_s3 = boto3.client("s3")
_BUCKET = os.environ["BUCKET"]
_PREFIX = os.environ.get("PREFIX", "telemetry").strip("/")
_SECRET = os.environ["API_KEY"]


def _header(headers, name):
    if not headers:
        return ""
    # Lambda Function URL delivers header names in lowercase.
    return headers.get(name) or headers.get(name.lower()) or ""


def _response(status, payload):
    return {
        "statusCode": status,
        "headers": {"Content-Type": "application/json"},
        "body": json.dumps(payload),
    }


def handler(event, _context):
    headers = event.get("headers") or {}
    if not hmac.compare_digest(_header(headers, "x-api-key"), _SECRET):
        return _response(403, {"error": "forbidden"})

    body = event.get("body") or ""
    if event.get("isBase64Encoded"):
        body = base64.b64decode(body).decode("utf-8")

    try:
        records = json.loads(body)
    except (ValueError, TypeError):
        return _response(400, {"error": "invalid json"})

    if isinstance(records, dict):
        records = [records]
    if not isinstance(records, list) or not records:
        return _response(400, {"error": "expected a non-empty array"})

    computer = "unknown"
    lines = []
    for record in records:
        if isinstance(record, dict) and record.get("computerId") is not None:
            computer = str(record["computerId"])
        lines.append(json.dumps(record, separators=(",", ":")))
    payload = ("\n".join(lines) + "\n").encode("utf-8")

    now = time.gmtime()
    key = "{prefix}/{y:04d}/{m:02d}/{d:02d}/{computer}-{ms}-{token}.ndjson".format(
        prefix=_PREFIX,
        y=now.tm_year,
        m=now.tm_mon,
        d=now.tm_mday,
        computer=computer,
        ms=int(time.time() * 1000),
        token=uuid.uuid4().hex[:8],
    )

    _s3.put_object(
        Bucket=_BUCKET,
        Key=key,
        Body=payload,
        ContentType="application/x-ndjson",
    )
    return _response(200, {"stored": len(records), "key": key})
