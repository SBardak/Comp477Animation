#include "animator.h"
#include "interpolation.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "splitstring.h"
using namespace std;

void Animator::HandlePlus()
{
	switch (_mode)
	{
		case AnimationMode::Animating:
			HandleAnimPlus();
			break;
		default:
		case AnimationMode::Editing:
			HandleEditPlus();
			break;
	}
}

void Animator::HandleMinus()
{
	switch (_mode)
	{
	case AnimationMode::Animating:
		HandleAnimMinus();
		break;
	default:
	case AnimationMode::Editing:
		HandleEditMinus();
		break;
	}
}

void Animator::HandleEditPlus()
{
	/* Too far */
	if ((_currentFrame + 1) >= _state.size())
		return;
	
	/* Change rotations for frame */
	++_currentFrame;

	PrintCurrentFrameEdit();
	SetPose();
}

void Animator::HandleEditMinus()
{
	if (_currentFrame <= 0)
		return;
	--_currentFrame;

	PrintCurrentFrameEdit();
	SetPose();
}

void Animator::PrintCurrentFrameEdit() const
{
	cout << "Keyframe [" << (_currentFrame + 0) << "/" << _state.size() << "]" << endl;
}
void Animator::PrintCurrentFrameAnim() const
{
	cout << "Frame [" << (_currentFrame * (_speed + 1)) + _animationFrame + 0
		<< "/" << ((_state.size() - 1) * _speed + _state.size())
		<< "]" << endl;
}


void Animator::HandleAnimPlus()
{
	// Solo keyframe
	if (_state.size() == 1)
		return;

	++_animationFrame;
	/*
	0			:First keyframe
	Speed + 1	:Last keyframe
	*/
	if (_animationFrame > _speed + 1)
	{
		++_currentFrame;
		// We start at 1 since 0 was done as the last frame
		// of the previous keyframe
		_animationFrame = 1;
	}

	if (_currentFrame + 1 >= _state.size())
	{
		_currentFrame = 0;
		_animationFrame = 0;
	}

	PrintCurrentFrameAnim();
	float percentage = (float)_animationFrame / (_speed + 1);
	HandleInterpolation(_currentFrame, _currentFrame + 1, percentage);
}

void Animator::HandleAnimMinus()
{
	// Solo keyframe
	if (_state.size() == 1)
		return;

	--_animationFrame;
	if (_animationFrame < 0)
	{
		--_currentFrame;
		_animationFrame = _speed;
	}

	if (_currentFrame < 0)
	{
		_animationFrame = _speed + 1;
		_currentFrame = (_state.size() - 2);
	}

	PrintCurrentFrameAnim();
	float percentage = 1 - (float)_animationFrame / (_speed + 1);
	HandleInterpolation(_currentFrame + 1, _currentFrame, percentage);
}

void Animator::HandleInterpolation(int start, int end, float percentage)
{
	switch (_i_mode)
	{
	default:
	case InterpolationMode::Matrix:
		for (int i = 0; i < d->mySkeleton.joints.size() - JOINT_START; ++i)
		{
			Quaternion q1 = _state[start].Rotations[i];
			Quaternion q2 = _state[end].Rotations[i];

			float mat[16];
			q1.ToMatrix(mat);

			float mat2[16];
			q2.ToMatrix(mat2);

			Interpolation in;
			in.MatrixLerp(mat, mat2, percentage, mat);

			//d->mySkeleton.joints[i].rotation.FromMatrix(mat);
			for (int j = 0; j < 10; ++j)
				if (j % 4 == 3)continue;
				else d->mySkeleton.joints[i + JOINT_START].local_t[j] = mat[j];

				float t = 0;
		}
		d->mySkeleton.updateGlobal(false);
		break;
	case InterpolationMode::Euler:
		for (int i = 0; i < d->mySkeleton.joints.size() - JOINT_START; ++i)
		{
			Quaternion q1 = _state[start].Rotations[i];
			Quaternion q2 = _state[end].Rotations[i];

			float mat[3];
			q1.ToEuler(mat);

			float mat2[3];
			q2.ToEuler(mat2);

			Interpolation in;
			in.EulerLerp(mat, mat2, percentage, mat);

			float tr[16];
			Quaternion t2;
			t2.FromEuler(mat);
			t2.ToMatrix(tr);

			//d->mySkeleton.joints[i].rotation.FromMatrix(mat);
			for (int j = 0; j < 10; ++j)
				if (j % 4 == 3)continue;
				else d->mySkeleton.joints[i + JOINT_START].local_t[j] = tr[j];

				float t = 0;
		}
		d->mySkeleton.updateGlobal(false);
		break;
	case InterpolationMode::QLERP:
		for (int i = 0; i < d->mySkeleton.joints.size() - JOINT_START; ++i)
		{
			Quaternion q1 = _state[start].Rotations[i];
			Quaternion q2 = _state[end].Rotations[i];
			Quaternion t2;

			Interpolation in;
			in.QuaternionLerp(q1, q2, percentage, &d->mySkeleton.joints[i + JOINT_START].rotation);
		}
		d->mySkeleton.updateGlobal();
		break;
	case InterpolationMode::QSLERP:
		for (int i = 0; i < d->mySkeleton.joints.size() - JOINT_START; ++i)
		{
			Quaternion q1 = _state[start].Rotations[i];
			Quaternion q2 = _state[end].Rotations[i];
			Quaternion t2;

			Interpolation in;
			in.QuaternionSLERP(q1, q2, percentage, &d->mySkeleton.joints[i + JOINT_START].rotation);
		}
		d->mySkeleton.updateGlobal();
		break;
	}
	d->updateVertices();
}
void Animator::SetPose()
{
	if (!(_currentFrame >= _state.size()))
	{
		for (int i = 0; i < d->mySkeleton.joints.size() - JOINT_START; ++i)
		{
			Quaternion q = _state[_currentFrame].Rotations[i];

			d->mySkeleton.joints[i + JOINT_START].rotation = q;
		}
	}
	d->mySkeleton.updateGlobal();
	d->updateVertices();
}


void Animator::AddKeyFrame(Skeleton* s)
{
	if (_mode != AnimationMode::Editing)
		return;

	AnimationState state;
	for (int i = 0; i < s->joints.size() - JOINT_START; ++i)
	{
		//Quaternion q;
		//q.FromMatrix(s->joints[i].local_t);
		//state.Rotations.push_back(q);
		state.Rotations.push_back(s->joints[i + JOINT_START].rotation);
	}

	// Add a new key frame at end
	if ((_currentFrame + 1) >= _state.size())
	{
		_state.push_back(state);
		++_currentFrame;
	}
	// Insert new keyframe
	else
	{
		std::vector<AnimationState>::iterator it;
		it = _state.begin();

		_state.insert(it + _currentFrame, state);
	}

	cout << "Added a new keyframe" << endl;
}
void Animator::ReplaceKeyFrame(Skeleton* s)
{
	if (_mode != AnimationMode::Editing)
		return;

	if (_currentFrame < 0 || _currentFrame >= _state.size())
		return;

	AnimationState state;
	for (int i = 0; i < s->joints.size() - JOINT_START; ++i)
	{
		state.Rotations.push_back(s->joints[i + JOINT_START].rotation);
	}
	
	_state[_currentFrame] = state;

	cout << "Replace keyframe " << (_currentFrame + 0) << endl;
}
void Animator::DeleteKeyFrame()
{
	if (_mode != AnimationMode::Editing)
		return;

	if (0 == _state.size())
		return;

	if (_currentFrame >= _state.size())
	{
		_state.pop_back();
		--_currentFrame;
	}
	else 
		_state.erase(_state.begin() + _currentFrame);

	// No keyframes left, reset rotations
	if (_state.size() == 0)
		for (int i = 0; i < d->mySkeleton.joints.size(); ++i)
			d->mySkeleton.joints[i].rotation = Quaternion();
	
	SetPose();

	cout << "Removed a keyframe" << endl;
}



void Animator::Animate()
{
	if (!autoPlay || _mode != AnimationMode::Animating)
		return;

	// Single or no keyframe, ignore
	if (_state.size() < 2)
		return;

	if (_currentFrame + 1 == _state.size())
	{
		_animationFrame = 0;
		_currentFrame = 0;
	}

	HandleAnimPlus();
}

void Animator::SaveToFile()
{
	ofstream saveFile("save.anim");
	if (saveFile.is_open())
	{
		for (int i = 0; i < _state.size(); ++i)
		{
			stringstream ss;
			ss << i;
			for (int j = 0; j < _state[i].Rotations.size(); ++j)
				ss << " " << _state[i].Rotations[j];
			ss << endl;
			saveFile << ss.str();
		}
		cout << "Save complete" << endl;
	}
	else
		cout << "Could not open save.anim" << endl;
}
void Animator::LoadFromFile(string file)
{
	std::string strRot;
	std::ifstream loadFile(file);

	_state.clear();

	if (loadFile.is_open())
	{
		while (std::getline(loadFile, strRot)) { //Read a line to build a bone
			std::vector<std::string> rotParams;
			splitstring splitStr(strRot);
			rotParams = splitStr.split(' ');

			AnimationState s;
			for (int t = 0; t < 17; ++t)
			{
				Quaternion temp;
				int i = t * 4 + 1;
				temp.w = std::atof(rotParams[i + 0].c_str());
				temp.x = std::atof(rotParams[i + 1].c_str());
				temp.y = std::atof(rotParams[i + 2].c_str());
				temp.z = std::atof(rotParams[i + 3].c_str());
				s.Rotations.push_back(temp);
			}
			if (std::atoi(rotParams[0].c_str()) != _state.size())
			{
				std::cout << "[Warning!!!] Bone index not match\n";
			}

			_state.push_back(s);
		}
		cout << "Load complete" << endl;

		_currentFrame = 0;
		SetPose();
	}
	else
		cout << "Could not open " << file << endl;
}





void Animator::IncreaseSpeed() 
{ 
	++_speed; 
	cout << "Frames in between: " << _speed << endl;
}
void Animator::DecreaseSpeed() 
{ 
	_speed = --_speed < 0 ? 0 : _speed; 
	cout << "Frames in between: " << _speed << endl;
}