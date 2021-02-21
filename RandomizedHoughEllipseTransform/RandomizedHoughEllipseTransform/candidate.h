#pragma once


struct CandidateInfo {
    int X;
    int Y;
    double SemiMajor;
    double SemiMinor;
    double Angle;
    double Score;
};


class Candidate {
public:
    CandidateInfo candidateInfo;

    Candidate(int centerX, int centerY, double semiMajor, double semiMinor, double angle) {
        this->candidateInfo.X = centerX;
        this->candidateInfo.Y = centerY;
        this->candidateInfo.SemiMajor = semiMajor;
        this->candidateInfo.SemiMinor = semiMinor;
        this->candidateInfo.Angle = angle;
        this->candidateInfo.Score = 1.0;
    }

    void operator<<=(Candidate& c) {
        this->candidateInfo.X = _averageInfo(this->candidateInfo.X, c.candidateInfo.X);
        this->candidateInfo.Y = _averageInfo(this->candidateInfo.Y, c.candidateInfo.Y);
        this->candidateInfo.SemiMajor = _averageInfo(this->candidateInfo.SemiMajor, c.candidateInfo.SemiMajor);
        this->candidateInfo.SemiMinor = _averageInfo(this->candidateInfo.SemiMinor, c.candidateInfo.SemiMinor);
        this->candidateInfo.Angle = _averageInfo(this->candidateInfo.Angle, c.candidateInfo.Angle);
        this->candidateInfo.Score += 1;
    }

    friend std::ostream& operator<<(std::ostream& out, const Candidate& c) {
        out << "Center: " << c.candidateInfo.X << "," << c.candidateInfo.X << "  ";
        out << "Semi axis: [" << c.candidateInfo.SemiMajor << ", " << c.candidateInfo.SemiMinor << "]  ";
        out << "Angle: " << c.candidateInfo.Angle << " ";
        out << "Score: " << c.candidateInfo.Score << " ";
        return out;
    }

    bool IsDistClose(Candidate& c) {
        double CenterDist = sqrt(pow((this->candidateInfo.X - c.candidateInfo.X), 2) + pow((this->candidateInfo.Y - c.candidateInfo.Y), 2));
        double AngleDiff = abs(this->candidateInfo.Angle - c.candidateInfo.Angle);
        double Angle180 = (c.candidateInfo.Angle > 0) ? c.candidateInfo.Angle - M_PI : c.candidateInfo.Angle + M_PI;
        double AngleDiff180 = abs(this->candidateInfo.Angle - Angle180);
        double AngleDist = std::min(AngleDiff, Angle180);
        double SemiMajorDist = abs(this->candidateInfo.SemiMajor - c.candidateInfo.SemiMajor);
        double SemiMinorDist = abs(this->candidateInfo.SemiMinor - c.candidateInfo.SemiMinor);
        if (CenterDist < 5 && AngleDist < M_PI / 18.0 && SemiMajorDist < 10 && SemiMinorDist < 10) {
            return true;
        }
        return false;
    }

private:
    double _averageInfo(double oldInfo, double newInfo) {
        return (newInfo + oldInfo * this->candidateInfo.Score) / (this->candidateInfo.Score + 1.0);
    }
};