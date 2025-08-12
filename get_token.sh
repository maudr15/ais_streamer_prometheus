#!/usr/bin/env bash
set -euo pipefail

# Fill these in:
CLIENT_ID="ID"
CLIENT_SECRET="PASS"

# Scope commonly used for AIS; change if your app needs a different scope.
SCOPE="ais"

TOKEN_URL="https://id.barentswatch.no/connect/token"
OUT_FILE="token.txt"

echo "Requesting token..."
RESP="$(curl -s -X POST "$TOKEN_URL" \
  -H "Content-Type: application/x-www-form-urlencoded" \
  --data-urlencode client_id=${CLIENT_ID} \
  --data-urlencode client_secret=${CLIENT_SECRET} \
  --data-urlencode scope=ais \
  --data-urlencode grant_type=client_credentials)"
  #--data "grant_type=client_credentials&client_id=${CLIENT_ID}&client_secret=${CLIENT_SECRET}&scope=${SCOPE}" \

TOKEN="$(echo "$RESP" | jq -r '.access_token')"


if [[ -z "$TOKEN" ]]; then
  echo "Failed to extract access_token. Response:"
  echo "$RESP"
  exit 1
fi

echo -n "$TOKEN" > "$OUT_FILE"
echo "Wrote access token to ${OUT_FILE}"
