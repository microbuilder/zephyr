.. _tfm_secure_inference:

TF-M Secure Provisioning Sample
############################

Overview
********

Provisioning enables an IoT end device to securely communicate with a cloud service.
It involves following steps:

- Generate a persistent key (secp256r1) in TF-M
- Generate device Certificate Signing Request (CSR)
- Send the device CSR to a Certification Authority (CA)
- Receive device signed device certificate from CA
- Store device certificate in TF-M protected storage

Currently, this sample demonstrates step 1 and 2.

Building and Running
********************

On Target
=========

Refer to :ref:`tfm_ipc` for detailed instructions.

On QEMU:
========

Refer to :ref:`tfm_ipc` for detailed instructions.

Sample Output
=============

   .. code-block:: console

      [Sec Thread] Secure image initializing!
      Booting TFM v1.4.1
      [Crypto] Dummy Entropy NV Seed is not suitable for production!
      *** Booting Zephyr OS build v2.7.99-890-g8a0707727a33  ***
      csr_subject_name: O=Linaro,CN=00010203-0405-6407-8809-0A0B0C0D0E0F
      [00:00:00.017,000] <inf> app: Initialising PSA crypto
      [00:00:00.018,000] <inf> app: PSA crypto init completed
      [00:00:00.019,000] <inf> app: Persisting SECP256R1 key as #1
      [00:00:00.565,000] <inf> app: Retrieving public key for key #1

               0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
      00000000 04 36 FB FB D9 F5 8E CE F9 D0 3E DC 2C 3F 40 52 .6........>.,?@R
      00000010 4E 91 51 CD 86 4B 84 F0 90 7D D1 EE 3C 20 06 1B N.Q..K...}..< ..
      00000020 5A DC 5A 2A C0 84 10 9C 11 23 50 C6 8D 82 77 A5 Z.Z*.....#P...w.
      00000030 C9 66 70 67 77 5E B1 47 C8 B1 3D EC E2 6E F6 54 .fpgw^.G..=..n.T
      00000040 8E                                              .

      [00:00:01.128,000] <inf> app: Adding subject name to CSR
      [00:00:01.129,000] <inf> app: Adding subject name to CSR completed
      [00:00:01.129,000] <inf> app: Adding EC key to PK container
      [00:00:01.131,000] <inf> app: Adding EC key to PK container completed
      [00:00:01.132,000] <inf> app: Create device Certificate Signing Request
      [00:00:02.329,000] <inf> app: Create device Certificate Signing Request completed
      [00:00:02.330,000] <inf> app: Certificate Signing Request:

      -----BEGIN CERTIFICATE REQUEST-----
      MIH9MIGiAgEAMEAxDzANBgNVBAoMBkxpbmFybzEtMCsGA1UEAwwkMDAwMTAyMDMt
      MDQwNS02NDA3LTg4MDktMEEwQjBDMEQwRTBGMFkwEwYHKoZIzj0CAQYIKoZIzj0D
      AQcDQgAENvv72fWOzvnQPtwsP0BSTpFRzYZLhPCQfdHuPCAGG1rcWirAhBCcESNQ
      xo2Cd6XJZnBnd16xR8ixPezibvZUjqAAMAwGCCqGSM49BAMCBQADSAAwRQIgDy8B
      EL47rGh8Fbc7UYwQhnJC+/0McB/DPgu3Ob1AtToCIQC99lzWe1zYtSbwyqSdYoTX
      0aUtDwPBLI3tTrw8pt4YQw==
      -----END CERTIFICATE REQUEST-----

      [00:00:02.332,000] <inf> app: Encoding CSR as json
      [00:00:02.335,000] <inf> app: Encoding CSR as json completed
      [00:00:02.336,000] <inf> app: Certificate Signing Request in JSON:

      {"CSR":"-----BEGIN CERTIFICATE REQUEST-----\nMIH9MIGiAgEAMEAxDzANBgNVBAoMBkxpbmFybzEtMCsGA1UEAwwkMDAwMTAyMDMt\nMDQwNS02NDA3LTg4MDktMEEwQjBDMEQwRTBGMFkwEwYHKoZIzj0CAQYIKoZIzj0D\nAQcDQgAENvv72fWOzvnQPtwsP0BSTpFRzYZLhPCQfdHuPCAGG1rcWirAhBCcESNQ\nxo2Cd6XJZnBnd16xR8ixPezibvZUjqAAMAwGCCqGSM49BAMCBQADSAAwRQIgDy8B\nEL47rGh8Fbc7UYwQhnJC+/0McB/DPgu3Ob1AtToCIQC99lzWe1zYtSbwyqSdYoTX\n0aUtDwPBLI3tTrw8pt4YQw==\n-----END CERTIFICATE REQUEST-----\n"}
