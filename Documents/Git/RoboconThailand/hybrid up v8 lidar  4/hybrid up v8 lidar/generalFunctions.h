/*
 * generalFunctions.h
 *
 * Created: 1/29/2016 2:42:39 PM
 *  Author: 
 */ 


#ifndef GENERALFUNCTIONS_H_
#define GENERALFUNCTIONS_H_

void constrain(float &val,float minVal,float maxVal)
{
	if(val<minVal)
	val=minVal;
	if(val>maxVal)
	val=maxVal;
}

#endif /* GENERALFUNCTIONS_H_ */