// HEADERS ---------------------------------------------------------------------------------------
/*
 * Determines what should be done from the synchronous code side.
 * This should be run regularly (in the 1-10 Hz range) in main to manage the TX/RX states
 */
void manageTXRXState(void);
/*
 * Safely sends (and prints to terminal) one CAN frame.
 */
uint8_t CAN_Send(uint8_t *data, uint8_t len);
/*
 * The entry-point for sending data via CAN ISO-TP.
 * Decides automatically whether to send a SF or FF.
 */
void CAN_SendSFFF(uint8_t *data, uint16_t length);
/*
 * Sends the next batch of CFs from the data.
 * Note: ensure that the TXBuffer has not been modified since the FF or previous CF was sent.
 */
void CAN_SendCFs();
/*
 * Helper function that determines the expected shape of the next incoming CF bundle.
 * Also determines (and stores in LSB) if at least 1 more CF bundle is required.
 */
void setCFsExpected(void);
/*
 * Takes action based on the RX frame type.
 */
void routeRX(void);


// CODE ------------------------------------------------------------------------------------------


/*
 * Determines what should be done from the synchronous code side.
 * This should be run regularly (in the 1-10 Hz range) in main to manage the TX/RX states
 */
void manageTXRXState(void)
{
	// Send data if not sending
	if (TXRXState == 0)
	{
		// Build test data
		myTXDataLength = 32;
		for(int i=0; i<32; i++)
		{
			myTXBuffer[i]=i;
		}
		// Send test data
		//CAN_SendSFFF(myTXBuffer, myTXDataLength);
	}
	else if (TXRXState == 1)
	{
		retries++;
		if (retries >= 10)
		{
			TXRXState = 0; // Allow PQ to retry send on next iteration
		}
	}
	else if (TXRXState == 2)
	{
		// do nothing
	}
	else if (TXRXState == 3)
	{
		// do nothing
	}
	else if (TXRXState == 4)
	{
		printf("RX Buffer:");
		for (uint16_t i = 0; i < myRXDataLength; i++)
		{
			printf(" i=%02d %02X,", i, myRXBuffer[i]);
		}
		printf("\n");
		TXRXState = 0;
	}
}


/*
 * Safely sends (and prints to terminal) one CAN frame.
 */
uint8_t CAN_Send(uint8_t *data, uint8_t len)
{
    if (len > 8) len = 8;
    txHeader.DLC = len;
    printf("checking CAN TX mailboxes.\r\n");
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, data, &txMailbox) == HAL_OK)
        {
            printf("CAN TX | ID: 0x%03lX | DLC: %d\r\n",
                   txHeader.StdId, txHeader.DLC);
        }
        else
        {
            printf("CAN TX ERROR\r\n");
        }
        return 1;
    }
    printf("CAN TX mailboxes full, failed to send data.\r\n");
    return 0;
}


/*
 * The entry-point for sending data via CAN ISO-TP.
 * Decides automatically whether to send a SF or FF.
 */
void CAN_SendSFFF(uint8_t *data, uint16_t length)
{
	uint8_t packet[8] = {0};
	myTXBufferIndex = 0;

	// Single Frame (SF): up to 7 bytes payload
	if (length <= 7)
	{
	    packet[0] = (0x0u << 4) | (uint8_t)(length & 0x0Fu);
	    memcpy(&packet[1], data, length);
	    CAN_Send(packet, 8);   // send full 8 bytes for consistent logging
	    TXRXState = 0; // Code for not actively sending out data
	    return;
	}
	// (End SF)

	// First Frame (FF): 12 bit length -> 4095 byte data limit
	if (length>4095){
		// ERROR: Above the first frame memory limit, return an handle error
		TXRXState = 0; // Code for not actively sending out data
		return;

	}

	// PCI: First Frame (1) + 12-bit length
	packet[0] = (0x1u << 4) | (uint8_t)((length >> 8) & 0x0Fu);
    packet[1] = (uint8_t)(length & 0xFFu);

    // First 6 data bytes go in bytes 2..7
	memcpy(&packet[2], &data[0], 6);
	myTXBufferIndex += 6;

	CAN_Send(packet, 8);
	TXRXState = 1; // Code for having sent FF
	// IMPORTANT: Will send CFs after FC received from receiver board
}


/*
 * Sends the next batch of CFs from the data.
 * Note: ensure that the TXBuffer has not been modified since the FF or previous CF was sent.
 */
void CAN_SendCFs()
{
	if ((canRX[0] & 0xF0) != 0x30) return; // Did not receive FC frame. CAN_SendCF was called incorrectly
	if (TXRXState != 1 && TXRXState != 2) return; // Doesn't have CFs to send.
	// Determine 0=continue, 1=wait, 2=abort
	uint8_t pcl = canRX[0] & 0x0F;
	if (pcl == 1)
	{
		return;
	}
	else if (pcl != 0)
	{
		TXRXState = 0; // Abort data send process
		return;
	}
	// If here, should continue sending bytes!
	TXRXState = 2; // Code for actively sending CFs
	// Parse send parameters
	uint8_t numFrames = canRX[1];
	uint8_t frameWait = canRX[2]; // Note this is technically incorrect ISO-TP, but okay for 10ms standard delay

	uint8_t sn = 1;

	for (uint8_t i = 0; i <= numFrames; i++) //  changed loop so data is sent when numframes is not equal to 0
	{
		printf("sending frame %d\r\n", i);
		uint8_t packet[8] = {0};

		// PCI: 2 then index
		packet[0] = (0x2u << 4) | sn;
		sn = (sn + 1) & 0x0F;
		if (sn == 0) sn = 1;

		// Prep next data bytes
		uint16_t numRemaining = myTXDataLength - myTXBufferIndex;
		uint8_t numSend = numRemaining > 7 ? 7 : numRemaining;
		memcpy(&packet[1], &myTXBuffer[myTXBufferIndex], numSend);
		myTXBufferIndex += numSend;

		CAN_Send(packet, 8);

		//HAL_Delay(frameWait); Our FCs will have no frame wait delay - they can arrive out of order!

		if (myTXBufferIndex >= myTXDataLength)
		{
			TXRXState = 0; // Done sending data!
			break;
			// Note: this doesn't reset the dataIndex pointer.
		}
	}
}


/*
 * Helper function that determines the expected shape of the next incoming CF bundle.
 * Also determines (and stores in LSB) if at least 1 more CF bundle is required.
 */
void setCFsExpected(void)
{
	myCFsExpected = 0;
	uint16_t bytes_expected = myRXDataLength - myRXBufferIndex;
	uint16_t numCFs = (bytes_expected + 6) / 7; // Number of CFs needed (ceil division)
	uint8_t numCFs_this_block = (numCFs > 15) ? 15 : numCFs; // Limit to current block (max 15)
	for (uint8_t k = 1; k <= numCFs_this_block; k++) // Set bits 1–numCFs_this_block
	{
	    myCFsExpected |= (uint16_t)(1u << k);
	}
	if (numCFs > 15) // If more CFs are needed beyond this block, set LSB
	{
	    myCFsExpected |= 0x1; // now should = 0xFFFF
	}
}


/*
 * Takes action based on the RX frame type.
 */
void routeRX(void)
{
	// Get Frame Type
	uint8_t frameType = canRX[0] >> 4;

	if (frameType == 0)
	{
		// Case incoming frame is SF
		myRXDataLength = canRX[0] & 0x0F;
		memcpy(myRXBuffer, &canRX[1], myRXDataLength);
		myRXBufferIndex = myRXDataLength;
		// The data portion of the SF should be in myRXBuffer
		// Consider validating that the SF has been digested properly here
	}
	else if (frameType == 1)
	{
		// Case incoming frame is FF
		TXRXState = 3;
		myRXDataLength = 0x0FFF & ((((uint16_t)canRX[0]) << 8) | (uint16_t)canRX[1]);
		memcpy(myRXBuffer, &canRX[2], 6);
		myRXBufferIndex = 6;
		// The data portion of the FF should be in myRXBuffer (no action)
		// Consider validating that the FF has been digested properly here
		// Decide how much data to expect from the next CF dump
		setCFsExpected();
		// Send FC Frame
		myCFsReceived = 0;
		uint8_t packet[8] = {0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // requests 15 CFs
		CAN_Send(packet, 8);
	}
	else if (frameType == 2)
	{
		// Case incoming frame is CF
		if (TXRXState != 3)
		{
			return; // Should not be receiving CFs at the moment
		}
		// ID the CF
		uint8_t CFsn = canRX[0] & 0x0F;
		// Store in buffer in correct location
		uint16_t write_index = myRXBufferIndex + 7u * (CFsn - 1);
		if (write_index >= myRXDataLength)
		{
			return;
		}
		uint16_t bytes_remaining = myRXDataLength - write_index;
		uint8_t bytes_to_copy = (bytes_remaining >= 7) ? 7 : (uint8_t)bytes_remaining;
		memcpy(&myRXBuffer[write_index], &canRX[1], bytes_to_copy);
		// Note received
		myCFsReceived |= (uint16_t)(1u << CFsn);
		// Check for message complete
		if (myCFsReceived == myCFsExpected)
		{
			TXRXState = 4;
		}
		// if received all CFs, send FC Frame
		else if (myCFsReceived == 0xFFFE && (myCFsExpected & 0x1)) // All received when highest 15 bits are 1
		{
			myRXBufferIndex += 105; // 105 = 15*7
			// Decide how much data to expect from the next CF dump
			setCFsExpected();
			// Send CF
			myCFsReceived = 0;
			uint8_t packet[8] = {0x30, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // requests 15 CFs
			CAN_Send(packet, 8);
		}
	}
	else if (frameType == 3)
	{
		// Case incoming message is FC
		// Flag the FC
		uint8_t FCflag = canRX[0] & 0x0F;
		// Case by case
		if (FCflag == 0)
		{
			// Case continue with CFs
			CAN_SendCFs();
		}
		else if (FCflag == 1)
		{
			// Case wait
			// do nothing. The state machine will use this for retry logic later
		}
		else if (FCflag == 2)
		{
			// Case abort
			TXRXState = 0;
		}
	}
}

