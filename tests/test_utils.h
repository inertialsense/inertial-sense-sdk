/**
 * @file test_utils.h 
 * @brief a collection of functions and classes that might be useful when writing/running unit tests
 *
 * @author Kyle Mallory on 1/18/24.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK_UNIT_TESTS_TEST_UTILS_H
#define IS_SDK_UNIT_TESTS_TEST_UTILS_H

/**
 * A macro to generate a random integer number between two values.
 */
#define RAND_RANGE(MIN, MAX) ( MIN + rand() / (RAND_MAX / (MAX - MIN + 1) + 1) )

/**
 * Generates a string of some arbitrary length, containing some number of random sentences/paragraphs of "Lorem Ipsum" text.
 * This is primarily used to generate files of random content.
 * @param minWords  the minimum number of words per sentence to generate
 * @param maxWords  the maximum number of words per sentence to generate
 * @param minSentences  the minimum number of sentences per paragraph to generate
 * @param maxSentences  the maximum number of sentences per paragraph to generate
 * @param numLines  the actual number of lines (paragraphs?) to generate (this is NOT random).
 * @return a std::string containing the generated content
 */
std::string LoremIpsum(int minWords, int maxWords, int minSentences, int maxSentences, int numLines);

#endif //IS_SDK_UNIT_TESTS_TEST_UTILS_H
