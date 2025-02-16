#include "subsystems.hpp"
#include "csort.hpp"

using namespace pros;

int sortingstate = 0;                        // Default sorting state
double minhue = 0.00;                        // Minimum hue value
double maxhue = 50.00;                       // Maximum hue value
double detectedhue = csortoptical.get_hue(); // Variable that returns optical sensor's detected hue value

void csort_task() {
    while(1) {
        double detectedhue = csortoptical.get_hue();
        
        if (sorting = 1) {
            minhue = 0.00;
            maxhue = 50.00;
        }
        else if (sorting = 2) {
            minhue = 220.00;
            maxhue = 270.00;
        }
        else {
            minhue = 0;
            maxhue = 0;
        }
        
        if ((sorting = 1) && ((minhue <= detectedhue) && ( detectedhue <= maxhue))) {
            csortpist.set(true);
            pros::delay(50);
        }
        else if ((sorting = 2) && ((minhue <= detectedhue) && ( detectedhue <= maxhue))) {
            csortpist.set(true);
            pros::delay(50);
        }
        else {
            csortpist.set(false);
            pros::delay(50);
        }
    }
}

