/*
	* Animation class
*/
#ifndef ANIMATOR_H
#define ANIMATOR_H

#include <vector>

#include "quaternion.h"
#include "defMesh.h"

enum AnimationMode
{
	Editing, Animating
};
enum InterpolationMode
{
	Matrix, Euler, QLERP, QSLERP
};

struct AnimationState
{
	std::vector<Quaternion> Rotations;
};

class Animator
{
private:
	AnimationMode _mode;
	InterpolationMode _i_mode;
	int _speed = 10, _currentFrame = -1, _animationFrame = 0;
	bool autoPlay = false;

	std::vector<AnimationState> _state;
	

	void HandleEditPlus();
	void HandleAnimPlus();
	void HandleEditMinus();
	void HandleAnimMinus();

	void HandleInterpolation(int start, int end, float percentage);
	void SetPose();

	const int JOINT_START = 1;

public:
	DefMesh *d;

	AnimationMode GetAnimationMode() { return _mode; }
	void SetAnimationMode(AnimationMode m) { _mode = m; Reset(); SetPose(); }

	void IncreaseSpeed();
	void DecreaseSpeed();

	void SetInterpolationMode(InterpolationMode m) { _i_mode = m; }


	void HandlePlus();
	void HandleMinus();

	void AddKeyFrame(Skeleton* s);
	void ReplaceKeyFrame(Skeleton* s);
	void DeleteKeyFrame();


	void Animate();

	void ToggleAutoPlay() { autoPlay = !autoPlay; }

	void SaveToFile();
	void LoadFromFile(std::string file);

	void PrintCurrentFrameEdit() const;
	void PrintCurrentFrameAnim() const;
	void Reset()
	{
		_currentFrame = 0;
		_animationFrame = 0;
	}
};
#endif
