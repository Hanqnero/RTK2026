# Arduino Build and Flash Cheatsheet

## Windows

Configure:

```powershell
cd D:\Dev\RTK\arduino
cmake --preset windows-bundled
```

Build:

```powershell
cmake --build build
```

Build and flash:

```powershell
cmake --build build --target flash
```

Configure with custom upload port:

```powershell
cmake --preset windows-bundled -DUPLOAD_PORT=COM5
```

Configure with explicit `avrdude` path:

```powershell
cmake --preset windows-bundled -DAVRDUDE_PROGRAM=C:\path\to\avrdude.exe
```

## macOS

Configure:

```bash
cd /path/to/RTK/arduino
cmake --preset macos-system
```

Build:

```bash
cmake --build build
```

Build and flash:

```bash
cmake --build build --target flash
```

Configure with custom upload port:

```bash
cmake --preset macos-system -DUPLOAD_PORT=/dev/cu.usbserial-110
```

## Notes

- The `flash` target depends on the firmware target, so it builds before uploading.
- If `avrdude` is not on `PATH`, set `AVRDUDE_PROGRAM` during configure.
- If your generator uses a different build directory, replace `build` in the commands above.
