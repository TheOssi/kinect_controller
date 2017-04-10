#include "fl/Headers.h"

using namespace fl;

struct resultSet {
	float backwardSpeed;
	float sidewardSpeed;
	float upSpeed;
	float rotationSpeed;
};

class FuzzyController {
private:
	Engine* engine;
	InputVariable* sideward;
	InputVariable* up;
	InputVariable* rotation;
	OutputVariable* backwardSpeed;
	OutputVariable* sidewardSpeed;
	OutputVariable* upSpeed;
	OutputVariable* rotationSpeed;

public:
	void init();
	resultSet getFISResult(float, float, float, float);

};
