#include "PID.h"
#include <iostream>
#include <math.h>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    
    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;
    
    cout << "Kp_: " << Kp_ << " Ki_: " << Ki_ << " Kd_: " << Kd_ << endl;
    
    p_error = 0.0;
    i_error = 0.0;
    d_error = 0.0;
    
    totalError_ = 0.0;
    bestError_ = std::numeric_limits<double>::max();
    
    iter_ = 0;
    settle_iter_ = 100;
    eval_iter_ = 500;
    
    coefficient_index_ = 0;
    should_add_ = true;
    should_subtract_ = true;
    
    _is_first_update = true;
    
    for (int i=0; i<3; i++) {
        if (i == 0) {
            dK_.push_back(0.1 * Kp_);
        } else if (i == 1) {
            dK_.push_back(0.1 * Ki_);
        } else {
            dK_.push_back(0.1 * Kd_);
        }
    }
    
    shouldApplyTwiddle_ = false;
}

void PID::UpdateError(double cte) {
    
    if (iter_ == 0) {
        p_error = cte;
    }
    
    double prev_cte = p_error;
    
    p_error = cte;
    i_error += cte;
    d_error =  cte - prev_cte;
    
    cout << "Kp_: " << Kp_ << " Ki_: " << Ki_ << " Kd_: " << Kd_ << endl;
    cout << "p_error: " << p_error << " i_error: " << i_error << " d_error: " << d_error << endl;
    
    if (iter_ % (settle_iter_ + eval_iter_) > settle_iter_) {
        totalError_ += pow(cte, 2);
    }
    
    if (iter_ % (settle_iter_ + eval_iter_) == 0) {
        
        // determine whether to apply twiddle
        double sum_dK_ = 0.0;
        for (int i=0; i<dK_.size(); i++) {
            sum_dK_ += dK_[i];
        }
        
        shouldApplyTwiddle_ = (sum_dK_ > 0.00001);
        
        if (shouldApplyTwiddle_) {
            
            UpdateCoefficientsUsingTwiddle(cte);
            
            cout << "Kp_: " << Kp_ << "\nKi_: " << Ki_ << "\nKd_: " << Kd_ << endl;
            for (int i=0; i<dK_.size(); i++) {
                cout << "dK_[" << i << "]: " << dK_[i] << endl;
            }
        }
        
        // reset totalError_
        totalError_ = 0.0;
    }
    
    cout << "iteration: " << iter_ << endl;
    
    iter_ ++;
}

double PID::TotalError() {
    double error = -Kp_*p_error - Ki_*i_error - Kd_*d_error;
    cout << "error: " << totalError_ << endl;
    return error;
}

void PID::UpdateCoefficientsUsingTwiddle(double cte) {
    
    cout << "totalError_: " << totalError_ << " bestError_: " << bestError_ << endl;
    if (totalError_ < bestError_) {
        
        cout << "Found an improvement!!!!" << endl;
        
        bestError_ = totalError_;
        
        if (_is_first_update == true) {
            _is_first_update = false;
        } else {
            dK_[coefficient_index_] *= 1.1;
        }
        
        // update the coefficient_index_
        coefficient_index_ = (coefficient_index_+ 1) % dK_.size();
        
        should_add_ = true;
        should_subtract_ = true;
    }
    
    if (should_add_ == true && should_subtract_ == true) {
        
        cout << "************* Adding for parameter at index: " << coefficient_index_ << endl;
        UpdateCoefficientByAddingDelta(coefficient_index_, dK_[coefficient_index_]);
        should_add_ = false;
        
    } else if (should_add_ == false && should_subtract_ == true) {
        
        cout << "************* Subtracting for parameter at index: " << coefficient_index_ << endl;
        UpdateCoefficientByAddingDelta(coefficient_index_, -2*dK_[coefficient_index_]);
        should_subtract_ = false;
        
    } else {
        
        cout << "Adding/Subtracting did not improve error for coefficient at index: " << coefficient_index_ << " Continue to next parameter." << endl;
        UpdateCoefficientByAddingDelta(coefficient_index_, dK_[coefficient_index_]);
        dK_[coefficient_index_] *= 0.9;
        
        // update the coefficient_index_
        coefficient_index_ = (coefficient_index_+ 1) % dK_.size();
        
        should_add_ = true;
        should_subtract_ = true;
    }
}

void PID::UpdateCoefficientByAddingDelta(int index, double delta) {
    
    if (index == 0) {
        Kp_ += delta;
        cout << "Updated Kp_: " << Kp_ << endl;
    } else if (index == 1) {
        Ki_ += delta;
        cout << "Updated Ki_: " << Ki_ << endl;
    } else if (index == 2) {
        Kd_ += delta;
        cout << "Updated Kd_: " << Kd_ << endl;
    } else {
        cout << "ERROR: unknown index found" << endl;
    }
}
