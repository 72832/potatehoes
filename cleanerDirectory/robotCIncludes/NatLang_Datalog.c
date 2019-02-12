#pragma once
#pragma systemFile            	// eliminates warning for "unreferenced" functions

typedef enum _tDatalogSeries {
	series_0 = 0,
	series_1,
	series_2,
	series_3,
	series_4,
	series_5,
	series_6,
	series_7,
} tDatalogSeries;

typedef enum _tSelectValue {
	kSelectValueNone = 0
} tSelectValue;

typedef enum _tDatalogUpdataFlags {
	kDatalogUpdateSeries_none = 0,
	kDatalogUpdateSeries_0 = (1<<0),
	kDatalogUpdateSeries_1 = (1<<1),
	kDatalogUpdateSeries_2 = (1<<2),
	kDatalogUpdateSeries_3 = (1<<3),
	kDatalogUpdateSeries_4 = (1<<4),
	kDatalogUpdateSeries_5 = (1<<5),
	kDatalogUpdateSeries_6 = (1<<6),
	kDatalogUpdateSeries_7 = (1<<7)
} tDatalogUpdataFlags;

static tDatalogUpdataFlags	_datalogUpdate;

// Storage for value yet to be written to the datalog
static long 								_datalogValue0;
static long 								_datalogValue1;
static long 								_datalogValue2;
static long 								_datalogValue3;
static long 								_datalogValue4;
static long 								_datalogValue5;
static long 								_datalogValue6;
static long 								_datalogValue7;

// current implementation does not really need a semaphore as
// values are both updated and written to the datalog by the same task
// if the user were to call datalogAddToSeries directly from code
// then semaphore is used to lock the resource
static TSemaphore						_datalogSemaphore;

// forward ref
// this function will be created by the graphical compiler
void _datalogUpdateAllvalues();

// Task that updates datalog in the background
task _datalogPollingTask()
{
		long		count = 0;
		long    deltaTime;
		long		lastRunTime = nSysTime;

		while(1)
			{
			// try and compensate for other tasks stopping us run
			// (weird code is because of compiler typecast bug)
			deltaTime = (nSysTime - lastRunTime);
			deltaTime = _datalogUpdateInterval - deltaTime;
			if( deltaTime < 10 ) deltaTime = 10;

			wait1Msec( deltaTime );
			lastRunTime = nSysTime;

			if( count < _datalogUpdatePeriod || _datalogUpdatePeriod < 0 )
				{
				count += deltaTime;

				// Call the function to get all user specified values
				_datalogUpdateAllvalues();

				// Now check flags and write updated values to the datalog
				// semaphore is used to protect the _datalogUpdate resource
				// and _datalogValueX global variables
				semaphoreLock( _datalogSemaphore, 10 );
				if( bDoesTaskOwnSemaphore(_datalogSemaphore) )
					{
					if( _datalogUpdate != kDatalogUpdateSeries_none )
						{
						datalogDataGroupStart();
						if(_datalogUpdate & kDatalogUpdateSeries_0)
							datalogAddValue( (short)series_0, _datalogValue0 );
						if(_datalogUpdate & kDatalogUpdateSeries_1)
							datalogAddValue( (short)series_1, _datalogValue1 );
						if(_datalogUpdate & kDatalogUpdateSeries_2)
							datalogAddValue( (short)series_2, _datalogValue2 );
						if(_datalogUpdate & kDatalogUpdateSeries_3)
							datalogAddValue( (short)series_3, _datalogValue3 );
						if(_datalogUpdate & kDatalogUpdateSeries_4)
							datalogAddValue( (short)series_4, _datalogValue4 );
						if(_datalogUpdate & kDatalogUpdateSeries_5)
							datalogAddValue( (short)series_5, _datalogValue5 );
						if(_datalogUpdate & kDatalogUpdateSeries_6)
							datalogAddValue( (short)series_6, _datalogValue6 );
						if(_datalogUpdate & kDatalogUpdateSeries_7)
							datalogAddValue( (short)series_7, _datalogValue7 );
						datalogDataGroupEnd();

						_datalogUpdate = kDatalogUpdateSeries_none;
						}
					semaphoreUnlock( _datalogSemaphore );
					}
				}
			}
}

// used for graphical block, not called directly
void
datalogAddToSeries( tDatalogSeries series = series_0, tSelectValue value = kSelectValueNone )
{
		return;
}

// Add one value to a datalog series
// The value is stored and written into the datalog the next time
// the above task is running
void
datalogAddToSeries( tDatalogSeries series = series_0, long longValue = 0 )
{
		if( getTaskState( _datalogPollingTask ) == taskStateStopped )
			{
			semaphoreInitialize( _datalogSemaphore );
			startTask( _datalogPollingTask, kDefaultTaskPriority + 2 );
			}

		semaphoreLock( _datalogSemaphore, 10 );

		if( bDoesTaskOwnSemaphore(_datalogSemaphore) )
			{
			switch(series)
				{
				case	series_0:	_datalogValue0 = longValue;	_datalogUpdate |= kDatalogUpdateSeries_0; break;
				case	series_1:	_datalogValue1 = longValue;	_datalogUpdate |= kDatalogUpdateSeries_1; break;
				case	series_2:	_datalogValue2 = longValue;	_datalogUpdate |= kDatalogUpdateSeries_2; break;
				case	series_3:	_datalogValue3 = longValue;	_datalogUpdate |= kDatalogUpdateSeries_3; break;
				case	series_4:	_datalogValue4 = longValue;	_datalogUpdate |= kDatalogUpdateSeries_4; break;
				case	series_5:	_datalogValue5 = longValue;	_datalogUpdate |= kDatalogUpdateSeries_5; break;
				case	series_6:	_datalogValue6 = longValue;	_datalogUpdate |= kDatalogUpdateSeries_6; break;
				case	series_7:	_datalogValue7 = longValue;	_datalogUpdate |= kDatalogUpdateSeries_7; break;
				default: break;
				}

			semaphoreUnlock( _datalogSemaphore );
			}
}

// datalogSetup in a special graphical function that when detected creates
// some header code in the source file.  The function is also left
// in the main task and is used to start the background task as necessary
// the parameters passed are not currently used.
void
datalogSetup( tDatalogSeries series = series_0, long longValue )
{
		if( getTaskState( _datalogPollingTask ) == taskStateStopped )
			{
			semaphoreInitialize( _datalogSemaphore );
			startTask( _datalogPollingTask, kDefaultTaskPriority + 2 );
			}
}
