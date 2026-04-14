# Install script for directory: /Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Users/adam/.espressif/tools/xtensa-esp32s3-elf/esp-2021r2-patch5-8.4.0/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/mbedtls" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/aes.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/aesni.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/arc4.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/aria.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/asn1.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/asn1write.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/base64.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/bignum.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/blowfish.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/bn_mul.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/camellia.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ccm.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/certs.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/chacha20.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/chachapoly.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/check_config.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/cipher.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/cipher_internal.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/cmac.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/compat-1.3.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/config.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/config_psa.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/constant_time.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ctr_drbg.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/debug.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/des.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/dhm.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ecdh.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ecdsa.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ecjpake.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ecp.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ecp_internal.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/entropy.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/entropy_poll.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/error.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/gcm.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/havege.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/hkdf.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/hmac_drbg.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/md.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/md2.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/md4.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/md5.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/md_internal.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/memory_buffer_alloc.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/net.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/net_sockets.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/nist_kw.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/oid.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/padlock.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/pem.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/pk.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/pk_internal.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/pkcs11.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/pkcs12.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/pkcs5.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/platform.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/platform_time.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/platform_util.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/poly1305.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/psa_util.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ripemd160.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/rsa.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/rsa_internal.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/sha1.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/sha256.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/sha512.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ssl.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ssl_cache.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ssl_ciphersuites.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ssl_cookie.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ssl_internal.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/ssl_ticket.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/threading.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/timing.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/version.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/x509.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/x509_crl.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/x509_crt.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/x509_csr.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/mbedtls/xtea.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/psa" TYPE FILE PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ FILES
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_builtin_composites.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_builtin_primitives.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_compat.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_config.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_driver_common.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_composites.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_driver_contexts_primitives.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_extra.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_platform.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_se_driver.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_sizes.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_struct.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_types.h"
    "/Users/adam/esp/esp-idf-v4.4.3/components/mbedtls/mbedtls/include/psa/crypto_values.h"
    )
endif()

