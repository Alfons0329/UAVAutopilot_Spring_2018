#include "pid.hpp"

PIDManager::PIDManager(string _filename) {
	/*
	 *	Initialize PIDManager with Coeffs from file
	 */
	importCoeffsFromFile(_filename);
	reset();
}

PIDManager::PIDManager(Mat& _X, Mat& _Y, Mat& _Z, Mat& _R) {
	/*
	 *	Initialize PIDManager with Coeffs
	 *	See Coeffs format in function setCoeffs()
	 */
	setCoeffs(_X, _Y, _Z, _R);
	reset();
}

void PIDManager::reset() {
	/*
	 *	Reset the errors
	 */
	mInit = false;
	error_integral = Mat::zeros(4, 1, CV_64F);
	previous_error = Mat::zeros(4, 1, CV_64F);
	previous_time = getCurrentTime();
}

void PIDManager::importCoeffsFromFile(string _filename) {
	/*
	 * 	Import coefficients of pid from _filename
	 */

	FileStorage fs(_filename, FileStorage::READ);
	fs["PID_X"] >> mX;
	fs["PID_Y"] >> mY;
	fs["PID_Z"] >> mZ;
	fs["PID_R"] >> mR;
	fs.release();
}

void PIDManager::setCoeffs(Mat& _X, Mat& _Y, Mat& _Z, Mat& _R) {
	/* 
	 * Mat _X(kp, ki, kd)
	 * Mat _Y(kp, ki, kd)
	 * Mat _Z(kp, ki, kd)
	 * Mat _R(kp, ki, kd)
	 */

	mX = _X.clone();
	mY = _Y.clone();
	mZ = _Z.clone();
	mR = _R.clone();
}

void PIDManager::getCommand(Mat& _error, Mat& _output) {
	/*
	 *	Input format : 	Mat(4, 1, CV_64F)
	 *		which stands for _error(x_error, y_error, z_error, r_error)
	 *	Output format : Mat(4, 1, CV_64F)
	 *		which stands for _output(x_out, y_out, z_out, r_out)
	 */
	
	double dt = (getCurrentTime() - previous_time) / 1000.; // in "sec" unit
	Mat de = Mat::zeros(4, 1, CV_64F);
	Mat output = Mat::zeros(4, 1, CV_64F);
	if(mInit) {
		for(int i = 0; i < 4; i++) {
			// de
			de.at<double>(i, 0) = (_error.at<double>(i, 0) - previous_error.at<double>(i, 0)) / dt;
			error_integral.at<double>(i, 0) += de.at<double>(i, 0) * dt;
		
			// output
			Mat coeffs;
			if(i == 0) 	coeffs = mX;
			else if(i == 1)	coeffs = mY;
			else if(i == 2) coeffs = mZ;
			else 		coeffs = mR;
			cout << endl << i << endl;
			cout << coeffs << endl;
			output.at<double>(i, 0) = _error.at<double>(i, 0) * coeffs.at<double>(0, 0)		// error * kp
						+ error_integral.at<double>(i, 0) * coeffs.at<double>(1, 0) 	// Sum(errors) * ki
						+ de.at<double>(i, 0) * coeffs.at<double>(2, 0);		// de * kd	
		}
	} else {
		mInit = true;
	}
	previous_error = _error.clone();
	_output = output.clone();
	previous_time = getCurrentTime();
}

double PIDManager::getCurrentTime() {
	/*
	 *	get current time (unit: ms)
	 */
	double t = 0.0;
#ifdef WINDOWS_IMPL
	DWORD time = GetTickCount();
	t = double(time);
#elif defined(UNIX_IMPL)
	struct timeval tv;
	gettimeofday(&tv, NULL);
	t = tv.tv_sec * 1000. + double(tv.tv_usec) / 1000.;
#endif
	return t;
}


