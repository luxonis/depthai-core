#include <iostream>

#include "depthai/depthai.hpp"

int main() {
    dai::DeviceBootloader::Config config;

    std::string ipv4 = "192.168.1.150";
    std::string ipv4Mask = "255.255.255.0";
    std::string ipv4Gateway = "192.168.1.1";

    config.setStaticIPv4(ipv4, ipv4Mask, ipv4Gateway);

    assert(ipv4 == config.getIPv4());
    assert(ipv4Mask == config.getIPv4Mask());
    assert(ipv4Gateway == config.getIPv4Gateway());

    std::string dns = "1.1.1.1";
    std::string dnsAlt = "8.8.8.8";

    config.setDnsIPv4(dns);

    assert(config.getDnsIPv4() == dns);
    assert(config.network.ipv4DnsAlt == 0);

    config.setDnsIPv4(dns, dnsAlt);

    assert(config.getDnsIPv4() == dns);
    assert(config.getDnsAltIPv4() == dnsAlt);

    // MAC address
    std::string mac = "FF:AA:BB:CC:00:11";
    config.setMacAddress(mac);
    // std::cout << "Orig mac address: " << mac << " len: " << mac.length() << std::endl;
    // std::cout << "Get  mac address: " << config.getMacAddress() << " len: " << config.getMacAddress().length() << std::endl;
    assert(config.getMacAddress() == mac);

    return 0;
}