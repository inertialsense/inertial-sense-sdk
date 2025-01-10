/**
 * @file test_ISFirmwarePackage.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 1/15/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include <fstream>
#include <iostream>

#include <gtest/gtest.h>
#include "gtest_helpers.h"
#include "test_utils.h"

#include "ISFileManager.h"
#include "ISFirmwareUpdater.h"


/**
 * This tests checks to that our zip/archive library for firmware packages works
 */
// The string to compress.
static const char *s_pTest_str =
        "MISSION CONTROL I wouldn't worry too much about the computer. First of all, there is still a chance that he is right, despite your tests, and" \
    "if it should happen again, we suggest eliminating this possibility by allowing the unit to remain in place and seeing whether or not it" \
    "actually fails. If the computer should turn out to be wrong, the situation is still not alarming. The type of obsessional error he may be" \
    "guilty of is not unknown among the latest generation of HAL 9000 computers. It has almost always revolved around a single detail, such as" \
    "the one you have described, and it has never interfered with the integrity or reliability of the computer's performance in other areas." \
    "No one is certain of the cause of this kind of malfunctioning. It may be over-programming, but it could also be any number of reasons. In any" \
    "event, it is somewhat analogous to human neurotic behavior. Does this answer your query?  Zero-five-three-Zero, MC, transmission concluded.";

TEST(ISFirmwarePackage, packages__assemble_archive) {
    int i;
    mz_bool status;
    const int N = 50;
    char data[2048];
    char hash_str[64];
    char archive_filename[64];
    md5hash_t md5sum;
    static const char *s_Test_archive_filename = "__fwUpdate.pkg";

    assert((strlen(s_pTest_str) + 64) < sizeof(data));

    TEST_COUT << "miniz.c version: " << MZ_VERSION << std::endl;
    TEST_COUT << "Working from directory: " << getcwd(data, sizeof(data)) << std::endl;

    // Delete the test archive, so it doesn't keep growing as we run this test
    remove(s_Test_archive_filename);

    // Append a bunch of text files to the test archive
    for (i = (N - 1); i >= 0; --i) {
        sprintf(archive_filename, "%u.txt", i);
        std::string content = LoremIpsum( 5, 35, 0, N - i, i);
        md5_hash(md5sum, content.length(), (uint8_t *)content.c_str());
        sprintf(hash_str, "%08x-%08x-%08x-%08x", md5sum.dwords[0], md5sum.dwords[1], md5sum.dwords[2], md5sum.dwords[3]);

        // Add a new file to the archive. Note this is an IN-PLACE operation, so if it fails your archive is probably hosed (its central directory may not be complete) but it should be recoverable using zip -F or -FF. So use caution with this guy.
        // A more robust way to add a file to an archive would be to read it into memory, perform the operation, then write a new archive out to a temp file and then delete/rename the files.
        // Or, write a new archive to disk to a temp file, then delete/rename the files. For this test this API is fine.
        status = mz_zip_add_mem_to_archive_file_in_place(s_Test_archive_filename, archive_filename, content.c_str(), content.length(), hash_str, (uint16_t) strlen(hash_str), MZ_BEST_COMPRESSION);
        ASSERT_TRUE(status) << "mz_zip_add_mem_to_archive_file_in_place failed!\n";
        //printf("Added file '%s' of %d bytes (with hash %s) to archive '%s')\n", archive_filename, content.length(), hash_str, s_Test_archive_filename);
    }

    // Add a directory entry for testing
    status = mz_zip_add_mem_to_archive_file_in_place(s_Test_archive_filename, "directory/", NULL, 0, "no comment", (uint16_t) strlen("no comment"), MZ_BEST_COMPRESSION);
    ASSERT_TRUE(status) << "mz_zip_add_mem_to_archive_file_in_place failed!\n";
    //printf("Added directory 'directory/' to archive '%s')\n", archive_filename, s_Test_archive_filename);
}

TEST(ISFirmwarePackage, packages__extract_archive) {
    int i;
    mz_bool status;
    size_t uncomp_size;
    mz_zip_archive zip_archive;
    void *p;
    char data[2048];
    // char hash_str[64];
    md5hash_t md5;
    static const char *s_Test_archive_filename = "__fwUpdate.pkg";

    TEST_COUT << "miniz.c version: " << MZ_VERSION << std::endl;
    TEST_COUT << "Working from directory: " << getcwd(data, sizeof(data)) << std::endl;

    // Now try to open the archive.
    memset(&zip_archive, 0, sizeof(zip_archive));

    status = mz_zip_reader_init_file(&zip_archive, s_Test_archive_filename, 0);
    ASSERT_TRUE(status) << "mz_zip_reader_init_file() failed!\n";

    // Get and print information about each file in the archive.
    for (i = 0; i < (int)mz_zip_reader_get_num_files(&zip_archive); i++)
    {
        mz_zip_archive_file_stat file_stat;
        status = mz_zip_reader_file_stat(&zip_archive, i, &file_stat);
        if (!status) {
            mz_zip_reader_end(&zip_archive);
            ASSERT_TRUE(status) << "mz_zip_reader_file_stat() failed!\n";
        }

        if (!strcmp(file_stat.m_filename, "directory/"))
        {
            status = mz_zip_reader_is_file_a_directory(&zip_archive, i);
            if (!status) {
                mz_zip_reader_end(&zip_archive);
                ASSERT_TRUE(status) << "'" << file_stat.m_filename << "' was expected to indicate 'is_file_a_directory', but was not.\n";
            }
        } else {
            // Try to extract all the files to the heap.
            p = mz_zip_reader_extract_file_to_heap(&zip_archive, file_stat.m_filename, &uncomp_size, 0);
            if (!p)
            {
                mz_zip_reader_end(&zip_archive);
                ASSERT_TRUE(p) << "Failed to extra content from archive for file '" << file_stat.m_filename << "'\n";
            }

            // validate the contents by calculating the MD5 of the data, and compare that with the hash stored in the comment for the file.
            md5_hash(md5, uncomp_size, (uint8_t*)p);
            std::string md5sum = md5_to_string(md5);
            std::string md5_comment(file_stat.m_comment, file_stat.m_comment_size);

            // Make sure the extraction really succeeded.
            status = (md5sum == md5_comment);
            if (status)
            {
                mz_free(p);
                mz_zip_reader_end(&zip_archive);
                ASSERT_TRUE(status) << "MD5sum mismatch in file '" << file_stat.m_filename << "': Expected: " << file_stat.m_comment << ", Actual: " << md5sum.c_str() << "\n";
            }

            // We're done.
            mz_free(p);
        }
    }

    // Close the archive, freeing any resources it was using
    mz_zip_reader_end(&zip_archive);

    // Clean up test file(s)
	ISFileManager::DeleteFile(s_Test_archive_filename);
}

#ifdef ZIP_PACKAGE_TEST
// this is a good test, but it requires a valid firmware package to run.  I should probably create a test to build a "test case" manifest and assemble in into a
// temporary package, but that's not going to happen today.
TEST(ISFirmwarePackage, parse_package) {
    dev_info_t devInfo = {};
    ISFirmwareUpdater* updater = new ISFirmwareUpdater(0, "/dev/ttyACM0", &devInfo);
    updater->openFirmwarePackage("/home/kylemallory/is-zephyr/is-gpx/firmware.pkg/IS_firmware_2.0.0.11.fpk");
    do {
        updater->fwUpdate_step();
    } while (!updater->fwUpdate_isDone());

    updater->cleanupFirmwarePackage();
    delete updater;
}
#endif

#ifdef MD5_PACKAGE_TEST
// this test is really more for my own sanity, and to ensure that we get matching MD5 checksums from the zips and we do from the actual files.
// It has a lot of specific dependencies (files, etc) so, its actually a REALLY BAD test.  Don't use it unless you are having problems with MD5 and Zips.
TEST(ISFirmwarePackage, package_file_md5) {
    mz_bool status;
    size_t uncomp_size;
    mz_zip_archive zip_archive;
    void *p;
    char data[2048];
    md5hash_t md5;
    const char* source_filename = "IS_GPX-1_zephyr_v2.0.0.11_b58_2024-01-23_174637.encrypted.bin";
    const char* archive_filename = "__test_archive.zip";
    const char* extracted_filename = "__test_original.raw";

    TEST_COUT << "miniz.c version: " << MZ_VERSION << std::endl;
    TEST_COUT << "Working from directory: " << getcwd(data, sizeof(data)) << std::endl;

    // first build and assemble the archive that we'll use for testing.
    // Delete the test archive, so it doesn't keep growing as we run this test
    remove(archive_filename);

    size_t fileSize;

    FILE *file = fopen(source_filename, "rb");
    ASSERT_NE(file, nullptr) << "Could not open file: " << source_filename << std::endl;

    fseek(file, 0, 2);
    fileSize = ftell(file);
    fseek(file, 0, 0);
    char* buff = (char*)malloc(fileSize);
    fread(buff, fileSize, 1, file);
    fclose(file);

    md5_hash(md5, fileSize, (uint8_t*)buff);
    printf("Original MD5 (raw-byte md5_hash): %s\n", md5_to_string(md5).c_str());

    md5Context_t ctx;
    md5_init(ctx);
    md5_update(ctx, (uint8_t*)buff, fileSize);
    md5_final(ctx, md5);
    printf("Original MD5 (raw-byte md5_update): %s\n", md5_to_string(md5).c_str());

    altMD5_reset();
    altMD5_hash(fileSize, (uint8_t*)buff);
    altMD5_getHash(md5);
    printf("Original MD5 (raw altMD5_hash): %s\n", md5_to_string(md5).c_str());

    free(buff);

    std::istream* fileIn = new std::ifstream(source_filename, std::ios::binary);
    md5_file_details(fileIn, fileSize, md5);
    printf("Original MD5 (md5_file_details): %s\n", md5_to_string(md5).c_str());
    delete fileIn;

    fileIn = new std::ifstream(source_filename, std::ios::binary);
    altMD5_file_details(fileIn, fileSize, md5);
    printf("Original MD5 (altMD5_file_details): %s\n", md5_to_string(md5).c_str());
    delete fileIn;

    // Create the new archive, and add out file...
    mz_zip_zero_struct(&zip_archive);
    status = mz_zip_writer_init_file(&zip_archive, archive_filename, 0);
    ASSERT_TRUE(status) << "Error creating test archive!\n";

    status = mz_zip_writer_add_file(&zip_archive, source_filename, source_filename, nullptr, 0, MZ_BEST_COMPRESSION);
    ASSERT_TRUE(status) << "Error adding content to archive!\n";

    status = mz_zip_writer_finalize_archive(&zip_archive);
    ASSERT_TRUE(status) << "Error finalizing test archive!\n";


    // Now try to open the archive.
    mz_zip_zero_struct(&zip_archive);
    status = mz_zip_reader_init_file(&zip_archive, archive_filename, 0);
    ASSERT_TRUE(status) << "mz_zip_reader_init_file() failed!\n";

    mz_zip_archive_file_stat file_stat;
    status = mz_zip_reader_file_stat(&zip_archive, 0, &file_stat);
    if (!status) {
        mz_zip_reader_end(&zip_archive);
        ASSERT_TRUE(status) << "mz_zip_reader_file_stat() failed!\n";
    }

    // Try to extract all the files to the heap.
    p = mz_zip_reader_extract_file_to_heap(&zip_archive, file_stat.m_filename, &uncomp_size, 0);
    if (!p)
    {
        mz_zip_reader_end(&zip_archive);
        ASSERT_TRUE(p) << "Failed to extract content from archive for file '" << file_stat.m_filename << "'\n";
    }

    FILE* of = fopen(extracted_filename, "wb");
    fwrite(p, uncomp_size, 1, of);
    fclose(of);


    md5Context_t md5Context;
    md5_init(md5Context);
    md5_update(md5Context, (uint8_t*)p, uncomp_size);
    md5_final(md5Context, md5);
    printf("Archive MD5 (raw-byte md5_update): %s\n", md5_to_string(md5).c_str());

    md5_hash(md5, uncomp_size, (uint8_t*)p);
    printf("Archive MD5 (raw-byte md5_hash): %s\n", md5_to_string(md5).c_str());

    altMD5_reset();
    altMD5_hash(uncomp_size, (uint8_t*)p);
    altMD5_getHash(md5);
    printf("Archive MD5 (raw altMD5_hash): %s\n", md5_to_string(md5).c_str());

    std::string casted_memory(static_cast<char*>(p), uncomp_size);
    fileIn = (std::istream*)new std::istringstream(casted_memory);
    md5_file_details(fileIn, fileSize, md5);
    printf("Archive MD5 (md5_file_details): %s\n", md5_to_string(md5).c_str());
    delete fileIn;

    fileIn = (std::istream*)new std::istringstream(casted_memory);
    altMD5_file_details(fileIn, fileSize, md5);
    printf("Archive MD5 (altMD5_file_details): %s\n", md5_to_string(md5).c_str());
    delete fileIn;

    fileIn = new std::ifstream(extracted_filename, std::ios::binary);
    md5_file_details(fileIn, fileSize, md5);
    printf("Extracted MD5 (md5_file_details): %s\n", md5_to_string(md5).c_str());
    delete fileIn;

    fileIn = new std::ifstream(extracted_filename, std::ios::binary);
    altMD5_file_details(fileIn, fileSize, md5);
    printf("Extracted MD5 (altMD5_file_details): %s\n", md5_to_string(md5).c_str());
    delete fileIn;

    // validate the contents by calculating the MD5 of the data, and compare that with the hash stored in the comment for the file.
    md5_hash(md5, uncomp_size, (uint8_t*)p);
    std::string md5sum = md5_to_string(md5);

    // We're done.
    mz_free(p);

    // Close the archive, freeing any resources it was using
    mz_zip_reader_end(&zip_archive);

    ASSERT_TRUE(status) << "MD5sum mismatch in file '" << file_stat.m_filename << "': Expected: " << file_stat.m_comment << ", Actual: " << md5sum.c_str() << "\n";
}
#endif