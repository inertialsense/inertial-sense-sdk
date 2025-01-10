/**
 * @file test_utils.cpp
 * @brief a collection of functions and classes that might be useful when writing/running unit tests
 *
 * @author Kyle Mallory on 1/18/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#include <string>
#include <vector>

#include "test_utils.h"


std::string LoremIpsum(int minWords, int maxWords, int minSentences, int maxSentences, int numLines)
{
    std::vector<std::string> words= {"lorem", "ipsum", "dolor", "sit", "amet", "consectetuer", "adipiscing", "elit", "sed", "diam", "nonummy", "nibh", "euismod", "tincidunt", "ut", "laoreet", "dolore", "magna", "aliquam", "erat"};
    int numSentences = RAND_RANGE(minSentences, maxSentences);
    int numWords = RAND_RANGE(minWords, maxWords);

    std::string sb;
    for (int p = 0; p < numLines; p++)
    {
        for (int s = 0; s < numSentences; s++)
        {
            for( int w = 0; w < numWords; w++ )
            {
                if( w > 0 ) { sb.append(" "); }
                int word_idx = RAND_RANGE(0, words.size() - 1);
                std::string word = words[ word_idx ];
                if( w == 0 ) { word[0] = toupper(word[0]); }
                sb.append( word );
            }
            sb.append(". ");
        }
        if ( p < numLines-1 ) sb.append("\n");
    }
    return sb;
}
