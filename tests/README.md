## Running tests

### All onhost tests

- These tests do NOT require a connected device (either via usb or poe)

```bash
ctest -L onhost # Run all tests runnable onhost
ctest -L usb_onhost # Run all tests runnable onhost with disabled POE
ctest -L poe_onhost # Run all tests runnable onhost with disabled USB
```

### All ondevice tests

- These tests require a connected device (either via usb or poe)

```bash
ctest -L ondevice # Run all tests runnable ondevice
ctest -L usb_ondevice # Run all tests runnable ondevice with disabled POE
ctest -L poe_ondevice # Run all tests runnable ondevice with disabled USB
```