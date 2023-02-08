/*
 * logManger.h
 *
 *  Created on: Nov 9, 2021
 *      Author: yakirhuri
 */

#ifndef INCLUDE_LOG_MANAGER_H
#define INCLUDE_LOG_MANAGER_H


#include <vector>
#include<string.h>
#include <tf/tf.h>
#include <iostream>
#include <fstream>

#include <angles/angles.h>




using namespace std;

class LogManager {



public:

	LogManager(string logPath = ""){		

		logPath_ = logPath;
	}

	virtual ~LogManager(){

		logFile_.close();
	}

	
	void setLogPath(const string& logPath) {

        logPath_ = logPath;

		string currTime = getCurrentTime();

		string full_path = logPath_ +"/"+currTime+".txt";
		cerr<<" ttttttttttttttthe path is "<<full_path<<endl;
		logFile_.open (full_path);
		
    }

	void writeToLog(const string& line){

		string currTime = getCurrentTime();

		logFile_ << currTime<<": "<<line<<endl;
	}

	void closeFile() {
		
		if( logFile_.is_open()){
			
			logFile_.close();
		}
	}

	
	
	

private:

	string getCurrentTime()
	{
		time_t rawtime;
		struct tm* timeinfo;
		char buffer[80];

		time(&rawtime);
		timeinfo = localtime(&rawtime);

		strftime(buffer, 80, "%F/%H_%M_%S", timeinfo);
		string curr_time_Str = strdup(buffer);
		std::replace(curr_time_Str.begin(), curr_time_Str.end(), '/', '_');
		std::replace(curr_time_Str.begin(), curr_time_Str.end(), '-', '_');

		return curr_time_Str;
	}

private:

	string logPath_ = "";

	ofstream logFile_;


};

#endif /* INCLUDE_LOG_MANAGER_H */
