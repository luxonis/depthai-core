import depthai as dai


def test_xlink_protocol_bindings_are_complete():
    protocols = (
        dai.XLinkProtocol.X_LINK_USB_VSC,
        dai.XLinkProtocol.X_LINK_USB_CDC,
        dai.XLinkProtocol.X_LINK_PCIE,
        dai.XLinkProtocol.X_LINK_IPC,
        dai.XLinkProtocol.X_LINK_TCP_IP,
        dai.XLinkProtocol.X_LINK_LOCAL_SHDMEM,
        dai.XLinkProtocol.X_LINK_TCP_IP_OR_LOCAL_SHDMEM,
        dai.XLinkProtocol.X_LINK_USB_EP,
        dai.XLinkProtocol.X_LINK_NMB_OF_PROTOCOLS,
        dai.XLinkProtocol.X_LINK_ANY_PROTOCOL,
    )

    for protocol in protocols:
        device_info = dai.DeviceInfo(
            "",
            "",
            dai.XLinkDeviceState.X_LINK_ANY_STATE,
            protocol,
            dai.XLinkPlatform.X_LINK_ANY_PLATFORM,
            dai.XLinkError_t.X_LINK_SUCCESS,
        )

        assert device_info.protocol == protocol
        assert "???" not in repr(device_info.protocol)
