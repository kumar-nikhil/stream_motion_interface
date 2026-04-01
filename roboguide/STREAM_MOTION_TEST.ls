/PROG  STREAM_MOTION_TEST
/ATTR
  OWNER		= MNEDITOR;
  COMMENT		= "Stream Motion Test Program";
  PROG_SIZE	= 128;
  CREATE	= DATE 25-01-01  TIME 00:00:00;
  MODIFIED	= DATE 25-01-01  TIME 00:00:00;
  FILE_NAME	= ;
  VERSION	= 0;
  LINE_COUNT	= 6;
  MEMORY_SIZE	= 256;
  PROTECT	= READ_WRITE;
  TCD:  STACK_SIZE	= 0,
        TASK_PRIORITY	= 50,
        TIME_SLICE	= 0,
        BUSY_LAMP_OFF	= 0,
        ABORT_REQUEST	= 0,
        PAUSE_REQUEST	= 0;
  DEFAULT_GROUP	= 1,*,*,*,*;
  CONTROL_CODE	= 00000000 00000000;
/MN
   1:  ;
   2:  ! Stream Motion Test ;
   3:  ! Manual: B-84904EN/01 Section 2.3 ;
   4:  ! Run in AUTO mode 100% override ;
   5:  IBGN start[1] ;
   6:  IBGN end[1] ;
/POS
/END
