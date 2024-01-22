/**
 * @file test_ISFirmwarePackage.cpp 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 1/15/24.
 * @copyright Copyright (c) 2024 Inertial Sense, Inc. All rights reserved.
 */

#include <gtest/gtest.h>
#include "gtest_helpers.h"
#include "test_utils.h"

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

TEST(ISFirmwareUpdate, packages__assemble_archive) {
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

TEST(ISFirmwareUpdate, packages__extract_archive) {
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
}

TEST(ISFirmwarePackage, parse_package) {
    dev_info_t devInfo;
    ISFirmwareUpdater* updater = new ISFirmwareUpdater(0, "/dev/ttyACM0", &devInfo);
    updater->openFirmwarePackage("/home/kylemallory/is-zephyr/is-gpx/firmware.pkg/IS_firmware_2.0.0.8.fpk");
    delete updater;
}