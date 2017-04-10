#include "FuzzyController.h"

void FuzzyController::init() {
	engine = new Engine;
	engine->setName("input");
	engine->setDescription("");
	InputVariable* backward = new InputVariable;
	backward->setName("backward");
	backward->setDescription("");
	backward->setEnabled(true);
	backward->setRange(-1.000, 1.000);
	backward->setLockValueInRange(false);
	backward->addTerm(
			new Trapezoid("strongForward", -2.000, -2.000, -0.800, -0.400));
	backward->addTerm(
			new Trapezoid("mediumForward", -0.900, -0.600, -0.400, 0.000));
	backward->addTerm(
			new Trapezoid("mediumBackward", 0.000, 0.400, 0.600, 0.900));
	backward->addTerm(
			new Trapezoid("strongBackward", 0.400, 0.800, 2.000, 2.000));
	backward->addTerm(new Trapezoid("zero", -0.600, -0.200, 0.200, 0.600));
	engine->addInputVariable(backward);

	sideward = new InputVariable;
	sideward->setName("sideward");
	sideward->setDescription("");
	sideward->setEnabled(true);
	sideward->setRange(-1.000, 1.000);
	sideward->setLockValueInRange(false);
	sideward->addTerm(
			new Trapezoid("strongLeft", -2.000, -2.000, -0.800, -0.400));
	sideward->addTerm(
			new Trapezoid("mediumLeft", -0.900, -0.600, -0.400, 0.000));
	sideward->addTerm(new Trapezoid("mediumRight", 0.000, 0.400, 0.600, 0.900));
	sideward->addTerm(new Trapezoid("strongRight", 0.400, 0.800, 2.000, 2.000));
	sideward->addTerm(new Trapezoid("zero", -0.600, -0.200, 0.200, 0.600));
	engine->addInputVariable(sideward);

	up = new InputVariable;
	up->setName("up");
	up->setDescription("");
	up->setEnabled(true);
	up->setRange(-1.000, 1.000);
	up->setLockValueInRange(false);
	up->addTerm(new Trapezoid("strongDown", -2.000, -2.000, -0.800, -0.400));
	up->addTerm(new Trapezoid("mediumDown", -0.900, -0.600, -0.400, 0.000));
	up->addTerm(new Trapezoid("mediumUp", 0.000, 0.400, 0.600, 0.900));
	up->addTerm(new Trapezoid("strongUp", 0.400, 0.800, 2.000, 2.000));
	up->addTerm(new Trapezoid("zero", -0.600, -0.200, 0.200, 0.600));
	engine->addInputVariable(up);

	rotation = new InputVariable;
	rotation->setName("rotation");
	rotation->setDescription("");
	rotation->setEnabled(true);
	rotation->setRange(-1.000, 1.000);
	rotation->setLockValueInRange(false);
	rotation->addTerm(
			new Trapezoid("strongLeft", -2.000, -2.000, -0.800, -0.400));
	rotation->addTerm(
			new Trapezoid("mediumLeft", -0.900, -0.600, -0.400, 0.000));
	rotation->addTerm(new Trapezoid("mediumRight", 0.000, 0.400, 0.600, 0.900));
	rotation->addTerm(new Trapezoid("strongRight", 0.400, 0.800, 2.000, 2.000));
	rotation->addTerm(new Trapezoid("zero", -0.600, -0.200, 0.200, 0.600));
	engine->addInputVariable(rotation);

	backwardSpeed = new OutputVariable;
	backwardSpeed->setName("backwardSpeed");
	backwardSpeed->setDescription("");
	backwardSpeed->setEnabled(true);
	backwardSpeed->setRange(-1.000, 1.000);
	backwardSpeed->setLockValueInRange(false);
	backwardSpeed->setAggregation(new UnboundedSum);
	backwardSpeed->setDefuzzifier(new Centroid(100));
	backwardSpeed->setDefaultValue(fl::nan);
	backwardSpeed->setLockPreviousValue(false);
	backwardSpeed->addTerm(
			new Triangle("strongForward", -1.200, -1.000, -0.700));
	backwardSpeed->addTerm(
			new Trapezoid("mediumForward", -1.000, -0.850, -0.600, -0.250));
	backwardSpeed->addTerm(
			new Trapezoid("mediumBackward", 0.250, 0.600, 0.850, 1.000));
	backwardSpeed->addTerm(new Triangle("strongBackward", 0.700, 1.000, 2.200));
	backwardSpeed->addTerm(new Trapezoid("zero", -0.800, -0.500, 0.500, 0.800));
	engine->addOutputVariable(backwardSpeed);

	sidewardSpeed = new OutputVariable;
	sidewardSpeed->setName("sidewardSpeed");
	sidewardSpeed->setDescription("");
	sidewardSpeed->setEnabled(true);
	sidewardSpeed->setRange(-1.000, 1.000);
	sidewardSpeed->setLockValueInRange(false);
	sidewardSpeed->setAggregation(new UnboundedSum);
	sidewardSpeed->setDefuzzifier(new Centroid(100));
	sidewardSpeed->setDefaultValue(fl::nan);
	sidewardSpeed->setLockPreviousValue(false);
	sidewardSpeed->addTerm(new Triangle("strongLeft", -1.200, -1.000, -0.700));
	sidewardSpeed->addTerm(
			new Trapezoid("mediumLeft", -1.000, -0.850, -0.600, -0.250));
	sidewardSpeed->addTerm(
			new Trapezoid("mediumRight", 0.250, 0.600, 0.850, 1.000));
	sidewardSpeed->addTerm(new Triangle("strongRight", 0.700, 1.000, 2.200));
	sidewardSpeed->addTerm(new Trapezoid("zero", -0.800, -0.500, 0.500, 0.800));
	engine->addOutputVariable(sidewardSpeed);

	upSpeed = new OutputVariable;
	upSpeed->setName("upSpeed");
	upSpeed->setDescription("");
	upSpeed->setEnabled(true);
	upSpeed->setRange(-1.000, 1.000);
	upSpeed->setLockValueInRange(false);
	upSpeed->setAggregation(new UnboundedSum);
	upSpeed->setDefuzzifier(new Centroid(100));
	upSpeed->setDefaultValue(fl::nan);
	upSpeed->setLockPreviousValue(false);
	upSpeed->addTerm(new Triangle("strongDown", -1.200, -1.000, -0.700));
	upSpeed->addTerm(
			new Trapezoid("mediumDown", -1.000, -0.850, -0.600, -0.250));
	upSpeed->addTerm(new Trapezoid("mediumUp", 0.250, 0.600, 0.850, 1.000));
	upSpeed->addTerm(new Triangle("strongUp", 0.700, 1.000, 2.200));
	upSpeed->addTerm(new Trapezoid("zero", -0.800, -0.500, 0.500, 0.800));
	engine->addOutputVariable(upSpeed);

	rotationSpeed = new OutputVariable;
	rotationSpeed->setName("rotationSpeed");
	rotationSpeed->setDescription("");
	rotationSpeed->setEnabled(true);
	rotationSpeed->setRange(-1.000, 1.000);
	rotationSpeed->setLockValueInRange(false);
	rotationSpeed->setAggregation(new UnboundedSum);
	rotationSpeed->setDefuzzifier(new Centroid(100));
	rotationSpeed->setDefaultValue(fl::nan);
	rotationSpeed->setLockPreviousValue(false);
	rotationSpeed->addTerm(new Triangle("strongLeft", -1.200, -1.000, -0.700));
	rotationSpeed->addTerm(
			new Trapezoid("mediumLeft", -1.000, -0.850, -0.600, -0.250));
	rotationSpeed->addTerm(
			new Trapezoid("mediumRight", 0.250, 0.600, 0.850, 1.000));
	rotationSpeed->addTerm(new Triangle("strongRight", 0.700, 1.000, 2.200));
	rotationSpeed->addTerm(new Trapezoid("zero", -0.800, -0.500, 0.500, 0.800));
	engine->addOutputVariable(rotationSpeed);

	RuleBlock* ruleBlock = new RuleBlock;
	ruleBlock->setName("");
	ruleBlock->setDescription("");
	ruleBlock->setEnabled(true);
	ruleBlock->setConjunction(new Minimum);
	ruleBlock->setDisjunction(new Maximum);
	ruleBlock->setImplication(new Minimum);
	ruleBlock->setActivation(new General);
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongForward then backwardSpeed is strongForward",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumForward then backwardSpeed is mediumForward",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumBackward then backwardSpeed is strongBackward",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongBackward then backwardSpeed is strongBackward",
					engine));
	ruleBlock->addRule(
			Rule::parse("if backward is zero then backwardSpeed is zero",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if sideward is strongLeft then sidewardSpeed is strongLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if sideward is mediumLeft then sidewardSpeed is mediumLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if sideward is mediumRight then sidewardSpeed is mediumRight",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if sideward is strongRight then sidewardSpeed is strongRight",
					engine));
	ruleBlock->addRule(
			Rule::parse("if sideward is zero then sidewardSpeed is zero",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumBackward then backwardSpeed is strongBackward",
					engine));
	ruleBlock->addRule(
			Rule::parse("if up is strongDown then upSpeed is strongDown",
					engine));
	ruleBlock->addRule(
			Rule::parse("if up is mediumDown then upSpeed is mediumDown",
					engine));
	ruleBlock->addRule(
			Rule::parse("if up is mediumUp then upSpeed is mediumUp", engine));
	ruleBlock->addRule(
			Rule::parse("if up is strongUp then upSpeed is strongUp", engine));
	ruleBlock->addRule(
			Rule::parse("if up is zero then upSpeed is zero", engine));
	ruleBlock->addRule(
			Rule::parse(
					"if rotation is strongLeft then rotationSpeed is strongLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if rotation is mediumLeft then rotationSpeed is mediumLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if rotation is mediumRight then rotationSpeed is mediumRight",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if rotation is strongRight then rotationSpeed is strongRight",
					engine));
	ruleBlock->addRule(
			Rule::parse("if rotation is zero then rotationSpeed is zero",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongForward and sideward is strongLeft then backwardSpeed is strongForward and sidewardSpeed is strongLeft and rotationSpeed is strongLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumForward and sideward is mediumLeft then backwardSpeed is mediumForward and sidewardSpeed is mediumLeft and rotationSpeed is mediumLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongForward and sideward is strongRight then backwardSpeed is strongForward and sidewardSpeed is strongRight and rotationSpeed is strongRight",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumForward and sideward is mediumRight then backwardSpeed is mediumForward and sidewardSpeed is mediumRight and rotationSpeed is mediumRight",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumForward and sideward is strongRight then backwardSpeed is mediumForward and sidewardSpeed is strongRight and rotationSpeed is strongRight",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumForward and sideward is strongLeft then backwardSpeed is mediumForward and sidewardSpeed is strongLeft and rotationSpeed is strongLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumForward and sideward is zero then backwardSpeed is mediumForward and sidewardSpeed is zero and rotationSpeed is zero",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongForward and sideward is zero then backwardSpeed is strongForward and sidewardSpeed is zero and rotationSpeed is zero",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongForward and sideward is mediumLeft then backwardSpeed is strongForward and sidewardSpeed is mediumLeft and rotationSpeed is strongLeft",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is strongForward and sideward is mediumRight then backwardSpeed is strongForward and sidewardSpeed is mediumRight and rotationSpeed is strongRight",
					engine));
	ruleBlock->addRule(
			Rule::parse(
					"if backward is mediumBackward then backwardSpeed is mediumBackward",
					engine));
	engine->addRuleBlock(ruleBlock);
}

resultSet FuzzyController::getFISResult(float back, float side, float upValue,
		float rotateRight) {

	resultSet result;
	//ROS_INFO("%.2f| %.2f | %.2f | %.2f", back, side, upValue, rotateRight);
	InputVariable* backward = engine->getInputVariable("backward");
	backward->setValue(back);
	sideward->setValue(side);
	up->setValue(upValue);
	rotation->setValue(rotateRight);

	engine->process();

	result.backwardSpeed = backwardSpeed->getValue();
	result.sidewardSpeed = sidewardSpeed->getValue();
	result.upSpeed = upSpeed->getValue();
	result.rotationSpeed = rotationSpeed->getValue();

	return result;
}
