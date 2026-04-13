# Red Light Box Center Test

This test harness runs the same red-cluster center logic used in
`arduino/esp32_serial_camera_sender/esp32_serial_camera_sender.ino`
against saved `red_light_box_*.jpg` images.

It loads a JPEG, converts it to RGB565, runs the cluster search over the
whole image, prints the chosen tracking pixel, and writes an annotated image
to `tests/red_light_box_center/out/`.

Example:

```bash
tests/red_light_box_center/run.sh trafficimages/red_light_box_65_1774889312867.jpg
```
