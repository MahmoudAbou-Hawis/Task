#include "APP_CFG.h"
#if CFG_IS_CURRENT_APP(MIAN_APP)

#include "App/App.h"

int main()
{
	System_Config();
	System_Start();
	while (1)
	{
	}

	return 0;
}

#endif