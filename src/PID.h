#ifndef PID_H
#define PID_H

#include <vector>

class PID {
public:
    /*
     * Errors
     */
    double p_error;
    double i_error;
    double d_error;
    
    /*
     * Coefficients
     */
    double Kp_;
    double Ki_;
    double Kd_;
    
    /*
     * Constructor
     */
    PID();
    
    /*
     * Destructor.
     */
    virtual ~PID();
    
    /*
     * Initialize PID.
     */
    void Init(double Kp, double Ki, double Kd);
    
    /*
     * Update the PID error variables given cross track error.
     */
    void UpdateError(double cte);
    
    /*
     * Calculate the total PID error.
     */
    double TotalError();
    
private:
    double totalError_;
    double bestError_;
    
    int iter_;
    int settle_iter_;
    int coefficient_index_;
    
    bool should_add_;
    bool should_subtract_;
    
    std::vector<double> dK_;
    
    bool shouldApplyTwiddle_;
    
    void UpdateCoefficientsUsingTwiddle(double cte);
    void UpdateCoefficientByAddingDelta(int index, double delta);
};

#endif /* PID_H */
