#pragma once


class CandidateInfo {
public:
    int centerX;
    int centerY;
    double semiMajor;
    double semiMinor;
    double angle;
    double score;
};


class Candidate {
public:
    CandidateInfo candidateInfo;

    Candidate(int centerX, int centerY, double semiMajor, double semiMinor, double angle) {
        this->candidateInfo.centerX = centerX;
        this->candidateInfo.centerY = centerY;
        this->candidateInfo.semiMajor = semiMajor;
        this->candidateInfo.semiMinor = semiMinor;
        this->candidateInfo.angle = angle;
        this->candidateInfo.score = 1.0;
    }

    void operator<<=(Candidate& c) {
        this->candidateInfo.centerX = _averageInfo(this->candidateInfo.centerX, c.candidateInfo.centerX);
        this->candidateInfo.centerY = _averageInfo(this->candidateInfo.centerY, c.candidateInfo.centerY);
        this->candidateInfo.semiMajor = _averageInfo(this->candidateInfo.semiMajor, c.candidateInfo.semiMajor);
        this->candidateInfo.semiMinor = _averageInfo(this->candidateInfo.semiMinor, c.candidateInfo.semiMinor);
        this->candidateInfo.angle = _averageInfo(this->candidateInfo.angle, c.candidateInfo.angle);
        this->candidateInfo.score += 1;
    }

    friend std::ostream& operator<<(std::ostream& out, const Candidate& c) {
        out << "Center: " << c.candidateInfo.centerX << "," << c.candidateInfo.centerX << "  ";
        out << "Semi axis: [" << c.candidateInfo.semiMajor << ", " << c.candidateInfo.semiMinor << "]  ";
        out << "Angle: " << c.candidateInfo.angle << " ";
        out << "Score: " << c.candidateInfo.score << " ";
        return out;
    }

    bool IsDistClose(Candidate& c) {
        double CenterDist = sqrt(pow((this->candidateInfo.centerX - c.candidateInfo.centerX), 2) + pow((this->candidateInfo.centerY - c.candidateInfo.centerY), 2));
        double AngleDiff = abs(this->candidateInfo.angle - c.candidateInfo.angle);
        double Angle180 = (c.candidateInfo.angle > 0) ? c.candidateInfo.angle - M_PI : c.candidateInfo.angle + M_PI;
        double AngleDiff180 = abs(this->candidateInfo.angle - Angle180);
        double AngleDist = std::min(AngleDiff, Angle180);
        double SemiMajorDist = abs(this->candidateInfo.semiMajor - c.candidateInfo.semiMajor);
        double SemiMinorDist = abs(this->candidateInfo.semiMinor - c.candidateInfo.semiMinor);
        if (CenterDist < 5 && AngleDist < M_PI / 18.0 && SemiMajorDist < 10 && SemiMinorDist < 10) {
            return true;
        }
        return false;
    }

private:
    double _averageInfo(double oldInfo, double newInfo) {
        return (newInfo + oldInfo * this->candidateInfo.score) / (this->candidateInfo.score + 1.0);
    }
};