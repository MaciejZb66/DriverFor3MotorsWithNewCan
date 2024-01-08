#include "main.h"
#include <math.h>
#include "Regulator.h"

void calcPWM(struct PI* regulation) {
	regulation->actualEvasion = regulation->requireValue-regulation->Vobr;
	regulation->S_k += ((KP_REG * KI_REG * regulation->actualEvasion)/TP);
	regulation->x = KP_REG * regulation->actualEvasion;
	regulation->S_k = fabsf(regulation->S_k) > MAX_VALUE_REG ? (copysignf(MAX_VALUE_REG, regulation->S_k)) : regulation->S_k;
	// Sprawdza czy zostala przekroczona maksymalna wartosc, jezeli tak to zapisuje max ze znakiem jaki ma ta zmienna
	regulation->PIoutput = regulation->x + regulation->S_k;

	regulation->PWM = regulation->PIoutput;
}


