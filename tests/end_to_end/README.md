# End To End

This runner replays the current BLE-image path on macOS using the same Edge Impulse model and the shared red-box evaluator.

Important limitation:
- it starts from BLE images, which are already the `160x160` EI model input
- so it is exact from `run_classifier(...)` onward
- it does not recreate the earlier full-frame camera crop step, because that information is not present in the BLE image alone

## Input

Put BLE images in:

`/Users/jamesbowler/esp32/tests/end_to_end/input`

Supported:
- `.jpg`
- `.jpeg`
- `.png`

Images are processed in filename sort order. If the filename ends in digits before the extension, those digits are used as `timestamp_ms` for the green-watch timing simulation.

## Run

```bash
zsh /Users/jamesbowler/esp32/tests/end_to_end/run.sh
```

Or with a custom input/output directory:

```bash
zsh /Users/jamesbowler/esp32/tests/end_to_end/run.sh /path/to/input /path/to/output
```

## Output

- console summary per frame
- annotated images in:
  - `/Users/jamesbowler/esp32/tests/end_to_end/out`

Annotations:
- yellow box: raw EI bounding box
- green box: adjusted padded/expanded box
- magenta box: bright core box used for centering
- red dot: tracked center
- cyan box: green-watch `10x10` search area
- black dot: green-watch origin pixel

