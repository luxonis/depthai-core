        #include <cmrc/cmrc.hpp>
#include <map>
#include <utility>

namespace cmrc {
namespace depthai {

namespace res_chars {
// These are the files which are available in this resource library
// Pointers to /home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/resources/depthai-device-fwp-5210eb74d850b630cb47f23e36f2a37c00a7feb2.tar.xz
extern const char* const f_ca96_depthai_device_fwp_5210eb74d850b630cb47f23e36f2a37c00a7feb2_tar_xz_begin;
extern const char* const f_ca96_depthai_device_fwp_5210eb74d850b630cb47f23e36f2a37c00a7feb2_tar_xz_end;
// Pointers to /home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/resources/depthai-bootloader-fwp-0.0.26.tar.xz
extern const char* const f_ef89_depthai_bootloader_fwp_0_0_26_tar_xz_begin;
extern const char* const f_ef89_depthai_bootloader_fwp_0_0_26_tar_xz_end;
// Pointers to /home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/resources/depthai-device-kb-fwp-0.0.1+fde4977d3dc1c66aa33fc0e81e6251022d4147b7.tar.xz
extern const char* const f_4365_depthai_device_kb_fwp_0_0_1_fde4977d3dc1c66aa33fc0e81e6251022d4147b7_tar_xz_begin;
extern const char* const f_4365_depthai_device_kb_fwp_0_0_1_fde4977d3dc1c66aa33fc0e81e6251022d4147b7_tar_xz_end;
// Pointers to /home/lan-sevcnikar/Documents/depthai/depthai-core/cmake-build-debug/resources/depthai-device-rvc4-fwp-0.0.1+09dfd074a6ea89e858b6d18d432add20783bb2d8.tar.xz
extern const char* const f_cea0_depthai_device_rvc4_fwp_0_0_1_09dfd074a6ea89e858b6d18d432add20783bb2d8_tar_xz_begin;
extern const char* const f_cea0_depthai_device_rvc4_fwp_0_0_1_09dfd074a6ea89e858b6d18d432add20783bb2d8_tar_xz_end;
}

namespace {

const cmrc::detail::index_type&
get_root_index() {
    static cmrc::detail::directory root_directory_;
    static cmrc::detail::file_or_directory root_directory_fod{root_directory_};
    static cmrc::detail::index_type root_index;
    root_index.emplace("", &root_directory_fod);
    struct dir_inl {
        class cmrc::detail::directory& directory;
    };
    dir_inl root_directory_dir{root_directory_};
    (void)root_directory_dir;
    
    root_index.emplace(
        "depthai-device-fwp-5210eb74d850b630cb47f23e36f2a37c00a7feb2.tar.xz",
        root_directory_dir.directory.add_file(
            "depthai-device-fwp-5210eb74d850b630cb47f23e36f2a37c00a7feb2.tar.xz",
            res_chars::f_ca96_depthai_device_fwp_5210eb74d850b630cb47f23e36f2a37c00a7feb2_tar_xz_begin,
            res_chars::f_ca96_depthai_device_fwp_5210eb74d850b630cb47f23e36f2a37c00a7feb2_tar_xz_end
        )
    );
    root_index.emplace(
        "depthai-bootloader-fwp-0.0.26.tar.xz",
        root_directory_dir.directory.add_file(
            "depthai-bootloader-fwp-0.0.26.tar.xz",
            res_chars::f_ef89_depthai_bootloader_fwp_0_0_26_tar_xz_begin,
            res_chars::f_ef89_depthai_bootloader_fwp_0_0_26_tar_xz_end
        )
    );
    root_index.emplace(
        "depthai-device-kb-fwp-0.0.1+fde4977d3dc1c66aa33fc0e81e6251022d4147b7.tar.xz",
        root_directory_dir.directory.add_file(
            "depthai-device-kb-fwp-0.0.1+fde4977d3dc1c66aa33fc0e81e6251022d4147b7.tar.xz",
            res_chars::f_4365_depthai_device_kb_fwp_0_0_1_fde4977d3dc1c66aa33fc0e81e6251022d4147b7_tar_xz_begin,
            res_chars::f_4365_depthai_device_kb_fwp_0_0_1_fde4977d3dc1c66aa33fc0e81e6251022d4147b7_tar_xz_end
        )
    );
    root_index.emplace(
        "depthai-device-rvc4-fwp-0.0.1+09dfd074a6ea89e858b6d18d432add20783bb2d8.tar.xz",
        root_directory_dir.directory.add_file(
            "depthai-device-rvc4-fwp-0.0.1+09dfd074a6ea89e858b6d18d432add20783bb2d8.tar.xz",
            res_chars::f_cea0_depthai_device_rvc4_fwp_0_0_1_09dfd074a6ea89e858b6d18d432add20783bb2d8_tar_xz_begin,
            res_chars::f_cea0_depthai_device_rvc4_fwp_0_0_1_09dfd074a6ea89e858b6d18d432add20783bb2d8_tar_xz_end
        )
    );
    return root_index;
}

}

cmrc::embedded_filesystem get_filesystem() {
    static auto& index = get_root_index();
    return cmrc::embedded_filesystem{index};
}

} // depthai
} // cmrc
    