vcpkg_check_linkage(ONLY_STATIC_LIBRARY)

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO rogersce/cnpy
    REF 4e8810b1a8637695171ed346ce68f6984e585ef4
    SHA512 fa19771d4cfc31edebf971ea89baf0242f2d8f2cba57fce0fd67e42378da169ac9872bdf3297ca1bb96f70358b0cbc3b36dfaba8bad9423776f75a7062cc824c
    HEAD_REF master
    PATCHES fix-size-t-accumulate.patch
)

file(COPY "${CMAKE_CURRENT_LIST_DIR}/CMakeLists.txt" DESTINATION "${SOURCE_PATH}")

vcpkg_cmake_configure(SOURCE_PATH "${SOURCE_PATH}")
vcpkg_cmake_install()
vcpkg_cmake_config_fixup(CONFIG_PATH share/cnpy)
vcpkg_copy_pdbs()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
