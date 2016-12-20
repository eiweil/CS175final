#include "cvec.h"

class Metaball
{
public:
  Cvec3 position;
  double radiusSquared;

  void init(Cvec3 newPos, double newRadiusSquared)
  {
    position = newPos;
    radiusSquared = newRadiusSquared;
  }
};