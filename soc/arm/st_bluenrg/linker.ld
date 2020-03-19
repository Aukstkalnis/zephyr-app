
#include <arch/arm/aarch32/cortex_m/scripts/linker.ld>

SECTIONS 
{
	#include <linker/rel-sections.ld>
	_MEMORY_RAM_BEGIN_   =  0x20000000; 
	_MEMORY_RAM_SIZE_    =  0x6000;           /* 24KB  */
	_MEMORY_RAM_END_     =  0x20005FFF; 

	_MEMORY_FLASH_BEGIN_ = 0x10040000;  
	_MEMORY_FLASH_SIZE_  = 0x40000;          /* 256KB */ 
	_MEMORY_FLASH_END_   = 0x1007FFFF;  

	_MEMORY_ROM_BEGIN_   = 0x10000000;    
	_MEMORY_ROM_SIZE_    = 0x800;             /* 2KB */ 
	_MEMORY_ROM_END_     = 0x100007FF;  
	FLASH_NVM_DATASIZE   = (4*1024);
	
	MEMORY_FLASH_APP_OFFSET = DEFINED(MEMORY_FLASH_APP_OFFSET) ? (MEMORY_FLASH_APP_OFFSET) : (0) ;
	MEMORY_FLASH_APP_SIZE = DEFINED(MEMORY_FLASH_APP_SIZE) ? (MEMORY_FLASH_APP_SIZE) : ( _MEMORY_FLASH_SIZE_ - FLASH_NVM_DATASIZE - MEMORY_FLASH_APP_OFFSET);
	MEMORY_RAM_APP_OFFSET = DEFINED(MEMORY_RAM_APP_OFFSET) ? (MEMORY_RAM_APP_OFFSET) : (0x2CC) ;
	
	_Min_Stack_Size = 0xC00; /* required amount of stack */
	/* RAM preamble no init */
	SECTION_PROLOGUE(.ram_preamble_noinit,(NOLOAD),)
	{
		. = ALIGN(4);
		. = 0x04 ; /* There is a waste of RAM here */
		KEEP(*(.ota_sw_activation))
	} GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)
	
	SECTION_PROLOGUE(.ram_preamble,(NOLOAD),)
	{
		_sdata2 = .;        /* create a global symbol at data start */
		/* This is used by the startup in order to initialize the .bss section */
		KEEP(*(.savedMSP))
		. = 0x04 ;
		KEEP(*(.wakeupFromSleepFlag))
		. = 0x08 ;
		KEEP(*(.app_base))
		. = 0x0C ;
		KEEP(*(.flash_sw_lock))
		. = 0x10;
		KEEP(*(.rfTimeout))
		. = 0x28 ;
		KEEP(*(.__blueflag_RAM))
		_edata2 = .;        /* create a global symbol at data end (.__crash_RAM is skipped since it must not be initialized) */
	} GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)

	SECTION_PROLOGUE(ram_preamble_noinit2,(NOLOAD),) 
	{
		. = 0x63;
		KEEP(*(.__crash_RAM))
	} GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)

	SECTION_PROLOGUE(.bss.blueRAM,(NOLOAD),) 
	{
		. = ALIGN(4);
    	_sbssblue = .;         /* define a global symbol at .bss.blueRAM start */
    	KEEP(*(.bss.__blue_RAM))
    	. = ALIGN(4);
    	_ebssblue = .;         /* define a global symbol at .bss.blueRAM end */
    	. = 0x20c + MEMORY_RAM_APP_OFFSET - 0x2cc;
	} GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)
	

	/* This is to emulate place at end of IAR linker */
	CSTACK (ORIGIN(RAMABLE_REGION) + LENGTH(RAMABLE_REGION) - _Min_Stack_Size) (NOLOAD) :
	{
		. = ALIGN(4);
		_estack = . + _Min_Stack_Size;         /* define a global symbol at bss end */
		. = ALIGN(4);
	} GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)

	_ebss = __bss_start;
	_sbss = __bss_start;
	_ebssblue = __bss_end;
	_sdata = __data_ram_start;
	/* used by the startup to initialize data */
	_sidata = LOADADDR(_DATA_SECTION_NAME);
	/* used by the startup to initialize data */
	_sidata2 = LOADADDR(.ram_preamble);
	_edata = __data_ram_end;
}