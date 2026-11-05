# Turns secrets.ini (gitignored) into -D defines for config.h, so the API key
# and MQTT credentials never have to be committed or typed into an escaped
# PLATFORMIO_BUILD_FLAGS string by hand.
#
#   [secrets]
#   DP_API_KEY = sk_...
#
# Every key becomes a string macro: DP_API_KEY -> -D DP_API_KEY=\"sk_...\".
# DP_MQTT_PORT is the one numeric setting and is passed through unquoted.
# A missing file is fine: config.h falls back to its defaults (uploads and
# MQTT disabled), except in the release env, which refuses to compile.

import configparser
import os

Import("env")  # noqa: F821 - provided by PlatformIO's SCons runtime

NUMERIC_KEYS = {"DP_MQTT_PORT"}

path = os.path.join(env.subst("$PROJECT_DIR"), "secrets.ini")  # noqa: F821

if os.path.isfile(path):
    # No interpolation: a '%' in a password must stay a '%'. Comments must be
    # on their own line - configparser keeps "value ; comment" as the value.
    parser = configparser.ConfigParser(interpolation=None)
    parser.optionxform = str  # keep the macro names' case
    parser.read(path, encoding="utf-8")

    if parser.has_section("secrets"):
        for key, value in parser.items("secrets"):
            value = value.strip()
            if not key.startswith("DP_") or not value:
                continue
            if " ;" in value or " #" in value or '"' in value or "\\" in value:
                raise ValueError(
                    "secrets.ini: %s contains a quote, a backslash or an inline comment; "
                    "put comments on their own line" % key
                )
            if key in NUMERIC_KEYS:
                if not value.isdigit():
                    raise ValueError("secrets.ini: %s must be a number, got %r" % (key, value))
                env.Append(CPPDEFINES=[(key, value)])  # noqa: F821
            else:
                env.Append(CPPDEFINES=[(key, env.StringifyMacro(value))])  # noqa: F821
        print("secrets.ini: loaded build secrets")
    else:
        print("secrets.ini: no [secrets] section, ignored")
else:
    print("secrets.ini: not found, uploads/MQTT use config.h defaults")
