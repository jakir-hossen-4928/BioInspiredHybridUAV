# Image quality and 4K settings

- Farm scene: `worlds/farm.wbt`
- Front RGB camera: 3840 x 2160 pixels
- Downward survey RGB camera: 3840 x 2160 pixels
- Saved camera pictures: PNG (lossless, no JPEG quality reduction)
- Local material maps: 4096 x 4096 PNG upscales
- True original material maps: `textures_original/` (source 1024-2048 pixels)

The materials have been upscaled; they do not contain additional
photographic source detail. Webots' main 3D viewport is controlled by
the monitor and window size. In full 4K mode, frame rate may decrease.
The existing navigation logic, 3D environment, sensors and original
algorithm limitations remain unchanged.
