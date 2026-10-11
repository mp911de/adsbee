# Sourced by build.sh; named profiles live beside this file.
export ADSBEE_PROFILE="${ADSBEE_PROFILE:-default}"
if [[ ! "$ADSBEE_PROFILE" =~ ^[a-z][a-z0-9_-]*$ ]] ||
   { [ "$ADSBEE_PROFILE" != default ] &&
     [ ! -f "$(dirname "${BASH_SOURCE[0]}")/$ADSBEE_PROFILE.cmake" ]; }; then
    echo "ERROR: Unknown ADSBEE_PROFILE: $ADSBEE_PROFILE" >&2
    return 1
fi

profile_ti_build_dir() {
    if [ "$ADSBEE_PROFILE" = default ]; then
        printf 'build'
    else
        printf 'build/%s-%s' "$1" "$ADSBEE_PROFILE"
    fi
}

profile_build_variant() {
    if [ "$ADSBEE_PROFILE" = default ]; then
        printf '%s' "$1"
    else
        printf '%s-%s' "$1" "$ADSBEE_PROFILE"
    fi
}
