#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include "../ISConstants.h"
#include "../util/md5.h"
using namespace std;


#if 1
TEST(md5, md5_hash)
{
    const char* data = "The quick brown fox jumped over the lazy dogs.\n";
    md5hash_t hash;

    md5_hash(hash, strlen(data), (uint8_t *) data);
    // md5_print(hash);    printf("\n");
    
    EXPECT_EQ(hash.dwords[0], SWAP32(0xb2616109));  // b261610902182c0581c791ac874f588c
    EXPECT_EQ(hash.dwords[1], SWAP32(0x02182c05));
    EXPECT_EQ(hash.dwords[2], SWAP32(0x81c791ac));
    EXPECT_EQ(hash.dwords[3], SWAP32(0x874f588c));
}
#endif

long GetFileSize(string filename)
{
    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

void check_md5_file(const char *filename)
{
    // Compute md5 hash using our library
    size_t filesize = 0;
    md5hash_t hash;
    EXPECT_EQ( md5_file_details(filename, filesize, hash.dwords), 0 );

    // Compare md5 hash using Linux md5sum app
    string hashStr = md5_string(hash); 
    std::stringstream cmd;
    cmd << "md5sum " << filename;
    FILE *fp = popen(cmd.str().c_str(), "r");
    char path[PATH_MAX];
    while (fgets(path, PATH_MAX, fp) != NULL)
    path[32] = 0;   // Null terminate to omit filename
    int status = pclose(fp);

    // Compare two hash strings
    EXPECT_EQ(hashStr, string(path));

    // Compare filesize
    EXPECT_EQ(filesize, GetFileSize(filename));

#if 0   // Print results
    printf("size: %d %s %s\n", (int)filesize, hashStr.c_str(), path);
#endif
}

#if 1
TEST(md5, file_md5_hash)
{

#if 1   // Repeat string with index count
    const char *filename = "md5_test.txt";
    // Create test file
    ofstream myfile(filename);
    for (int i=0; i<1024; i++)
    {
        myfile << "The quick brown fox jumped over the lazy dogs.\n";
        myfile << "Hope Kyle doesn't see this! " << i << "\n";
    }
    myfile.close();
#else
    string filename = "IS_IMX-5.bin";
#endif

    check_md5_file(filename);
}
#endif

#if 1
TEST(md5, md5_1_to_400k_file_size)
{
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

