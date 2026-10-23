#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_build_dir=$(mktemp -d /tmp/pid-host-tests.XXXXXX)
trap 'rm -rf -- "$test_build_dir"' EXIT
compiler=${CC:-gcc}
common=(-std=c11 -Wall -Wextra -Werror -O2)
for module in pid cmd vofa; do
  "$compiler" "${common[@]}" -I "App/$module" "tests/test_$module.c" "App/$module/$module.c" -lm -o "$test_build_dir/$module"
  "$test_build_dir/$module"
done
"$compiler" "${common[@]}" -I tests/stubs -I App/pid -I App/cmd -I App/speed-loop \
  tests/test_speed_loop.c App/speed-loop/speed-loop.c App/pid/pid.c App/cmd/cmd.c -lm -o "$test_build_dir/speed_loop"
"$test_build_dir/speed_loop"
"$compiler" "${common[@]}" -I tests/stubs -I App/ui -I App/speed-loop -I Components/oled-ssd1306 \
  tests/test_ui.c App/ui/ui.c Components/oled-ssd1306/oled_font.c -o "$test_build_dir/ui"
"$test_build_dir/ui"
