#include "gtest/gtest.h"
#include <deque>
#include "../ring_buffer.h"
#include "../ISComm.h"

#define BUFFER_SIZE	1024

TEST(RingBuffer, U8_1for1_Test)
{
	uint8_t buffer[BUFFER_SIZE] = { 0 };
	ring_buf_t rb;
	std::deque<uint8_t> myDeque;

	ringBufInit(&rb, buffer, sizeof(buffer));

	uint8_t val, ref;
	for (uint8_t i = 0; i < 255; i++)
	{
		ringBufWrite(&rb, &i, 1);
		myDeque.push_back(i);

		ringBufRead(&rb, &val, 1);
		ref = myDeque.front();
		myDeque.pop_front();
		EXPECT_EQ(val, ref);
	}
}


TEST(RingBuffer, U32_1for1_Test)
{
	uint8_t buffer[BUFFER_SIZE];
	ring_buf_t rb;
	std::deque<uint32_t> myDeque;

	ringBufInit(&rb, buffer, sizeof(buffer));

	uint32_t val, ref;
	for (uint32_t i = 0; i < 1000000; i++)
	{
		ringBufWrite(&rb, (uint8_t*)&i, 4);
		myDeque.push_back(i);

		ringBufRead(&rb, (uint8_t*)&val, 4);
		ref = myDeque.front();
		myDeque.pop_front();
		EXPECT_EQ(val, ref);
	}

	// Ensure buffer is empty
	EXPECT_TRUE(ringBufEmpty(&rb) == true && myDeque.empty() == true);
}


TEST(RingBuffer, WriteReadWriteReadTest)
{
	uint8_t buffer[BUFFER_SIZE];
	ring_buf_t rb;
	std::deque<uint8_t> myDeque;

	ringBufInit(&rb, buffer, BUFFER_SIZE);

	uint8_t val, ref;

	// Write 201
	for (uint8_t i = 0; i < 201; i++)
	{
		ringBufWrite(&rb, &i, 1);
		myDeque.push_back(i);
	}

	// Read 50 (Leaving 151)
	for (uint8_t i = 0; i < 50; i++)
	{
		ringBufRead(&rb, &val, 1);
		ref = myDeque.front();
		myDeque.pop_front();
		EXPECT_EQ(val, ref);
	}

	// Write 104 causing 255 total in buffer
	for (uint8_t i = 0; i < 104; i++)
	{
		ringBufWrite(&rb, &i, 1);
		myDeque.push_back(i);
	}

	// Read 254, leaving one in the buffer
	for (uint8_t i = 0; i < 254; i++)
	{
		ringBufRead(&rb, &val, 1);
		ref = myDeque.front();
		myDeque.pop_front();
		EXPECT_EQ(val, ref);
	}

	// Ensure buffer has 1
	EXPECT_TRUE(ringBufUsed(&rb) == 1 && myDeque.size() == 1);

	// Remove last element
	ringBufRead(&rb, &val, 1);
	ref = myDeque.front();
	myDeque.pop_front();

	// Ensure buffer is empty
	EXPECT_TRUE(ringBufEmpty(&rb)==true && myDeque.empty()==true);
}


TEST(RingBuffer, U32TestWriteThenReadTest)
{
	uint32_t buffer[BUFFER_SIZE];
	ring_buf_t rb;
	std::deque<uint32_t> myDeque;

	ringBufInit(&rb, (uint8_t*)buffer, sizeof(buffer));

	uint32_t val, ref;
	for (uint32_t i = 0; i < BUFFER_SIZE-1; i++)
	{
		ringBufWrite(&rb, (uint8_t*)&i, 4);
		myDeque.push_back(i);
	}

	for (uint32_t i = 0; i < BUFFER_SIZE-1; i++)
	{
		ringBufRead(&rb, (uint8_t*)&val, 4);
		ref = myDeque.front();
		myDeque.pop_front();

		EXPECT_EQ(val, ref);
	}
}


TEST(RingBuffer, PacketTest)
{
#define COMM_BUFFER_SIZE        2048
	static is_comm_instance_t   comm;
	static uint8_t              comm_buffer[COMM_BUFFER_SIZE];
	comm.buffer = comm_buffer;
	comm.bufferSize = COMM_BUFFER_SIZE;
	is_comm_init(&comm);

	ins_1_t ins1 = { 0 };
	int n;

	uint8_t buffer[BUFFER_SIZE];
	ring_buf_t rb;
	std::deque<uint8_t> myDeque;

	ringBufInit(&rb, buffer, sizeof(buffer));

	// Write until buffer is full
	for (int i = 0;; i++)
	{
		ins1.timeOfWeek = (double)i;
		ins1.week = i;
		ins1.insStatus = i;
		ins1.hdwStatus = i;
		ins1.theta[0] = i * 2.0f;
		ins1.theta[1] = i * 3.0f;
		ins1.theta[2] = i * 4.0f;
		ins1.uvw[0] = i * 5.0f;
		ins1.uvw[1] = i * 6.0f;
		ins1.uvw[2] = i * 7.0f;
		ins1.lla[0] = 40.330565516;
		ins1.lla[1] = -111.725787806;
		ins1.lla[2] = 1408.565264;
		ins1.ned[0] = i * 1.234f;
		ins1.ned[1] = i * 2.345f;
		ins1.ned[2] = i * 3.456f;

		n = is_comm_data(&comm, DID_INS_1, 0, sizeof(ins_1_t), (void*)&(ins1));

		if (n > ringBufFree(&rb))
		{	// Buffer is full

			// Check that ring buffer and deque has save amount
			EXPECT_EQ(ringBufUsed(&rb), myDeque.size());
			break;
		}

		ringBufWrite(&rb, comm.buffer, n);
		for (int j = 0; j < n; j++)
		{
			myDeque.push_back(comm.buffer[j]);
		}
	}

	// Read until buffer is empty
	uint8_t val;
	while (1)
	{
		if (ringBufEmpty(&rb) || myDeque.empty())
		{	// Buffer is empty

			// Ensure buffer is empty
			EXPECT_TRUE(ringBufEmpty(&rb) == true && myDeque.empty() == true);
			break;
		}

		ringBufRead(&rb, (uint8_t*)&val, 1);
		EXPECT_EQ(val, myDeque.front());
		myDeque.pop_front();
	}

}


TEST(RingBuffer, WriteOverflowTest)
{
#define BUFFERX_SIZE	1000
#define OVERLAP_SIZE	450
	uint8_t buffer[BUFFERX_SIZE];
	ring_buf_t rb;

	ringBufInit(&rb, buffer, BUFFERX_SIZE);

	// Buffer should one less than was allocated
	EXPECT_TRUE(ringBufFree(&rb) == BUFFERX_SIZE-1);

	// Write BUFFER_X_SIZE + OVERLAP_SIZE
	for (uint32_t i = 0; i < BUFFERX_SIZE+OVERLAP_SIZE; i++)
	{
		ringBufWrite(&rb, (uint8_t*)&i, 1);
	}

	// Ensure buffer should have BUFFER_X_SIZE-1 bytes
	EXPECT_TRUE(ringBufUsed(&rb) == BUFFERX_SIZE-1);

	// Check that overlap happened correctly
	uint8_t val;
	for (uint32_t i = OVERLAP_SIZE+1; ringBufUsed(&rb); i++)
	{
		ringBufRead(&rb, &val, 1);
		uint8_t ii = i;
		EXPECT_EQ(ii, val);
	}

	// Ensure buffer is empty
	EXPECT_TRUE(ringBufEmpty(&rb) == true);
}

