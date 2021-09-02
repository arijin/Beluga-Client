#ifndef _COORDINATE_TRANS_H_
#define _COORDINATE_TRANS_H_

double radians(double degree);
double degrees(double radian);
void toXYCoordinate(double lat1, double long1, double lat2, double long2, double &x, double &y);

#endif