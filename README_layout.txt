说明：两种结构都能用 PlatformIO 编译

Variant A（保持 CubeMX 结构）
- 目录：Core/Src, Core/Inc
- platformio.ini: src_dir=Core/Src, include_dir=Core/Inc
- 适合继续用 CubeMX 改引脚后再 Generate Code

Variant B（PlatformIO 经典结构）
- 目录：src/, include/
- 之后若用 CubeMX 改引脚，需要手动把 Core/Src/Inc 的变更再拷到 src/include
- 更贴近 PIO 习惯，但和 CubeMX 的自动生成耦合较弱