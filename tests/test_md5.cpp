#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include "../ISConstants.h"
#include "../util/md5.h"

#ifdef _WIN32
#include <windows.h>
#define popen _popen
#define pclose _pclose
#else
#include <unistd.h>
#endif

using namespace std;


long GetFileSize(string filename)
{
    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

void check_md5_file(const char *filename)
{
    // Compute MD5 hash using our library
    size_t filesize = 0;
    md5hash_t hash;
    std::ifstream s(filename, std::ios::binary);
    if (!s) {
        throw std::runtime_error("Failed to open file for reading");
    }

    EXPECT_EQ(md5_file_details(&s, filesize, hash), 0);

    // Convert computed hash to string
    std::string hashStr = md5_to_string(hash);

    // Compare MD5 hash using platform-specific command
    std::string cmdOutput;
    std::stringstream cmd;

#ifdef _WIN32
    cmd << "CertUtil -hashfile \"" << filename << "\" MD5";
#else
    cmd << "md5sum \"" << filename << "\"";
#endif

    FILE *fp = popen(cmd.str().c_str(), "r");
    if (!fp) {
        throw std::runtime_error("Failed to execute command");
    }

    char buffer[512];
    while (fgets(buffer, sizeof(buffer), fp) != nullptr) {
        cmdOutput += buffer;
    }
    pclose(fp);

#ifdef _WIN32
    // Extract MD5 hash from CertUtil output
    size_t start = cmdOutput.find(": ") + 2;
    size_t end = cmdOutput.find('\r', start); // CertUtil output may have \r\n
    cmdOutput = cmdOutput.substr(start, end - start);
#else
    // Extract MD5 hash from md5sum output
    cmdOutput = cmdOutput.substr(0, 32);
#endif

    // Remove any trailing whitespace
    cmdOutput.erase(cmdOutput.find_last_not_of(" \n\r\t") + 1);

    // Compare two hash strings
    EXPECT_EQ(hashStr, cmdOutput);

    // Compare file size
    struct stat fileStat;
    if (stat(filename, &fileStat) != 0) {
        throw std::runtime_error("Failed to retrieve file size");
    }
    EXPECT_EQ(filesize, static_cast<size_t>(fileStat.st_size));

#if 0   // Print results
    printf("size: %d %s %s\n", static_cast<int>(filesize), hashStr.c_str(), cmdOutput.c_str());
#endif
}

#ifndef _WIN32  // This test does not run correctly in Windows and needs to be debugged. (WHJ)

TEST(md5, hash)
{
    // GTEST_SKIP();
    const char* data = "The quick brown fox jumped over the lazy dogs.\n";
    md5hash_t hash;
    md5_hash(hash, strlen(data), (uint8_t *) data);

    EXPECT_EQ(hash.dwords[0], SWAP32(0xb2616109));  // b261610902182c0581c791ac874f588c
    EXPECT_EQ(hash.dwords[1], SWAP32(0x02182c05));
    EXPECT_EQ(hash.dwords[2], SWAP32(0x81c791ac));
    EXPECT_EQ(hash.dwords[3], SWAP32(0x874f588c));
}

TEST(md5, hash_to_string_to_hash)
{
    // GTEST_SKIP();
    const char* data = "The quick brown fox jumped over the lazy dogs.\n";
    md5hash_t hash;
    md5_hash(hash, strlen(data), (uint8_t *) data);
    string str = md5_to_string(hash);
    md5hash_t hash2 = md5_from_string(str);

    for (int i=0; i<4; i++)
    {
        EXPECT_EQ(hash.dwords[i], hash2.dwords[i]);
    }
 }

TEST(md5, hash_from_file)
{
    // GTEST_SKIP();
    const char *filename = "md5_test.txt";
    // Create test file
    ofstream myfile(filename);
    for (int i=0; i<1024; i++)
    {
        myfile << "The quick brown fox jumped over the lazy dogs.\n";
        myfile << "Hope Kyle doesn't see this! " << i << "\n";
    }
    myfile.close();

    check_md5_file(filename);
}

TEST(md5, hash_from_file_1_to_400k_file_size)
{
    // GTEST_SKIP();
    const char *filename = "md5_test.txt";

    for (int len=1; len<400000; )
    {
        // Create test file
        ofstream myfile(filename);
        for (int i=0; i<len; i++)   // broken
        {
            int v = i%10;
            myfile << v;
        }
        myfile.close();

        check_md5_file(filename);

        // Increment by 1/16
        len += _MAX(1, len>>4);
    }

    // Remove test file
    remove(filename);
}

#endif
