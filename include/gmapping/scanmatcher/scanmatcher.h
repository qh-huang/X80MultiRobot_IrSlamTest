//#include "../StdAfx.h"
#ifndef SCANMATCHER_H
#define SCANMATCHER_H

#include "icp.h"
#include "smmap.h"
#include <gmapping/utils/macro_params.h>
#include <gmapping/utils/stat.h>
#include <iostream>
#include <gmapping/utils/gvalues.h>
#include "Infrared.h"
#include "../openslam_gmapping/scanmatcher/gridlinetraversal.h"

#define LASER_MAXBEAMS 2048
#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))
#define NUM_IR 6
//#define NUM_IR 1

namespace GMapping {

	class ScanMatcher {
	public:
		typedef Covariance3 CovarianceMatrix;

		ScanMatcher();
		~ScanMatcher();
		double icpOptimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		double optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		double optimize(OrientedPoint& mean, CovarianceMatrix& cov, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		
		// qiao@2015.08.24: optimize function for IR reading
		double optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const Infrared* ir_readings) const;

		double   registerScan(ScanMatcherMap& map, const OrientedPoint& p, const double* readings);
		
		// qiao@2015.08.24: optimize function for IR reading
		double   registerScan(ScanMatcherMap& map, const OrientedPoint& p, const Infrared* readings);

		void setLaserParameters
			(unsigned int beams, double* angles, const OrientedPoint& lpose);
		void setMatchingParameters
			(double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations, double likelihoodSigma = 1, unsigned int likelihoodSkip = 0);
		void invalidateActiveArea();
		void computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const double* readings);
		void computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const Infrared* ir_readings);

		inline double icpStep(OrientedPoint & pret, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		inline double score(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		inline double score(const ScanMatcherMap & map, const OrientedPoint & p, const Infrared * ir_readings) const;
		inline double score_ScanMatching(const ScanMatcherMap & map, const OrientedPoint & p, const Infrared * ir_readings) const;
		inline unsigned int likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const;
		double likelihood(double& lmax, OrientedPoint& mean, CovarianceMatrix& cov, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings);
		double likelihood(double& _lmax, OrientedPoint& _mean, CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& p, Gaussian3& odometry, const double* readings, double gain = 180.);
		inline const double* laserAngles() const { return m_laserAngles; }
		inline unsigned int laserBeams() const { return m_laserBeams; }

		static const double nullLikelihood;
	protected:
		//state of the matcher
		bool m_activeAreaComputed;

		/**laser parameters*/
		unsigned int m_laserBeams;
		double       m_laserAngles[LASER_MAXBEAMS];
		//OrientedPoint m_laserPose;
		PARAM_SET_GET(OrientedPoint, laserPose, protected, public, public)
		PARAM_SET_GET(double, laserMaxRange, protected, public, public)
		/**scan_matcher parameters*/
		PARAM_SET_GET(double, usableRange, protected, public, public)
		PARAM_SET_GET(double, gaussianSigma, protected, public, public)
		PARAM_SET_GET(double, likelihoodSigma, protected, public, public)
		PARAM_SET_GET(int, kernelSize, protected, public, public)
		PARAM_SET_GET(double, optAngularDelta, protected, public, public)
		PARAM_SET_GET(double, optLinearDelta, protected, public, public)
		PARAM_SET_GET(unsigned int, optRecursiveIterations, protected, public, public)
		PARAM_SET_GET(unsigned int, likelihoodSkip, protected, public, public)
		PARAM_SET_GET(double, llsamplerange, protected, public, public)
		PARAM_SET_GET(double, llsamplestep, protected, public, public)
		PARAM_SET_GET(double, lasamplerange, protected, public, public)
		PARAM_SET_GET(double, lasamplestep, protected, public, public)
		PARAM_SET_GET(bool, generateMap, protected, public, public)
		PARAM_SET_GET(double, enlargeStep, protected, public, public)
		PARAM_SET_GET(double, fullnessThreshold, protected, public, public)
		PARAM_SET_GET(double, angularOdometryReliability, protected, public, public)
		PARAM_SET_GET(double, linearOdometryReliability, protected, public, public)
		PARAM_SET_GET(double, freeCellRatio, protected, public, public)
		PARAM_SET_GET(unsigned int, initialBeamsSkip, protected, public, public)
		// allocate this large array only once
		IntPoint* m_linePoints;
	};

	inline double ScanMatcher::icpStep(OrientedPoint & pret, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const {
		const double * angle = m_laserAngles + m_initialBeamsSkip;
		OrientedPoint lp = p;
		lp.x += cos(p.theta)*m_laserPose.x - sin(p.theta)*m_laserPose.y;
		lp.y += sin(p.theta)*m_laserPose.x + cos(p.theta)*m_laserPose.y;
		lp.theta += m_laserPose.theta;
		unsigned int skip = 0;
		double freeDelta = map.getDelta()*m_freeCellRatio;
		std::list<PointPair> pairs;

		for (const double* r = readings + m_initialBeamsSkip; r<readings + m_laserBeams; r++, angle++) {
			skip++;
			skip = skip>m_likelihoodSkip ? 0 : skip;
			if (*r>m_usableRange || *r == 0.0) continue;
			if (skip) continue;
			Point phit = lp;
			phit.x += *r*cos(lp.theta + *angle);
			phit.y += *r*sin(lp.theta + *angle);
			IntPoint iphit = map.world2map(phit);
			Point pfree = lp;
			pfree.x += (*r - map.getDelta()*freeDelta)*cos(lp.theta + *angle);
			pfree.y += (*r - map.getDelta()*freeDelta)*sin(lp.theta + *angle);
			pfree = pfree - phit;
			IntPoint ipfree = map.world2map(pfree);
			bool found = false;
			Point bestMu(0., 0.);
			Point bestCell(0., 0.);
			for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++)
				for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++) {
					IntPoint pr = iphit + IntPoint(xx, yy);
					IntPoint pf = pr + ipfree;
					//AccessibilityState s=map.storage().cellState(pr);
					//if (s&Inside && s&Allocated){
					const PointAccumulator& cell = map.cell(pr);
					const PointAccumulator& fcell = map.cell(pf);
					if (((double)cell)> m_fullnessThreshold && ((double)fcell)<m_fullnessThreshold) {
						Point mu = phit - cell.mean();
						if (!found) {
							bestMu = mu;
							bestCell = cell.mean();
							found = true;
						}
						else
							if ((mu*mu)<(bestMu*bestMu)) {
								bestMu = mu;
								bestCell = cell.mean();
							}

					}
					//}
				}
			if (found) {
				pairs.push_back(std::make_pair(phit, bestCell));
				//std::cerr << "(" << phit.x-bestCell.x << "," << phit.y-bestCell.y << ") ";
			}
			//std::cerr << std::endl;
		}

		OrientedPoint result(0, 0, 0);
		//double icpError=icpNonlinearStep(result,pairs);
		std::cerr << "result(" << pairs.size() << ")=" << result.x << " " << result.y << " " << result.theta << std::endl;
		pret.x = p.x + result.x;
		pret.y = p.y + result.y;
		pret.theta = p.theta + result.theta;
		pret.theta = atan2(sin(pret.theta), cos(pret.theta));
		return score(map, p, readings);
	}

	inline double ScanMatcher::score(const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const {
		double s = 0;
		const double * angle = m_laserAngles + m_initialBeamsSkip;
		OrientedPoint lp = p;
		lp.x += cos(p.theta)*m_laserPose.x - sin(p.theta)*m_laserPose.y;
		lp.y += sin(p.theta)*m_laserPose.x + cos(p.theta)*m_laserPose.y;
		lp.theta += m_laserPose.theta;
		unsigned int skip = 0;
		double freeDelta = map.getDelta()*m_freeCellRatio;
		for (const double* r = readings + m_initialBeamsSkip; r<readings + m_laserBeams; r++, angle++) {
			skip++;
			skip = skip>m_likelihoodSkip ? 0 : skip;
			if (skip || *r>m_usableRange || *r == 0.0) continue;
			Point phit = lp;
			phit.x += *r*cos(lp.theta + *angle);
			phit.y += *r*sin(lp.theta + *angle);
			IntPoint iphit = map.world2map(phit);
			Point pfree = lp;
			pfree.x += (*r - map.getDelta()*freeDelta)*cos(lp.theta + *angle);
			pfree.y += (*r - map.getDelta()*freeDelta)*sin(lp.theta + *angle);
			pfree = pfree - phit;
			IntPoint ipfree = map.world2map(pfree);
			bool found = false;
			Point bestMu(0., 0.);
			for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++)
				for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++) {
					IntPoint pr = iphit + IntPoint(xx, yy);
					IntPoint pf = pr + ipfree;
					//AccessibilityState s=map.storage().cellState(pr);
					//if (s&Inside && s&Allocated){
					const PointAccumulator& cell = map.cell(pr);
					const PointAccumulator& fcell = map.cell(pf);
					if (((double)cell)> m_fullnessThreshold && ((double)fcell)<m_fullnessThreshold) {
						Point mu = phit - cell.mean();
						if (!found) {
							bestMu = mu;
							found = true;
						}
						else
							bestMu = (mu*mu)<(bestMu*bestMu) ? mu : bestMu;
					}
					//}
				}
			if (found)
				s += exp(-1. / m_gaussianSigma*bestMu*bestMu);
		}
		return s;
	}

	// qiao@2015.08.24: score function for IR readings
	inline double ScanMatcher::score(const ScanMatcherMap& map, const OrientedPoint& p, const Infrared* ir_readings) const 
	{
		double s = 0;
		OrientedPoint ip = p; // IR pose
		double freeDelta = map.getDelta()*m_freeCellRatio;
		//for (size_t i = 0; i < NELEMS(ir_readings); i++) {
		for (size_t i = 0; i < NUM_IR; i++) {
			Point phit = ip;
			double angle = ir_readings[i].m_pose.theta;
			double r = ir_readings[i].m_Range;
			phit.x += r * cos(ip.theta + angle);
			phit.y += r * sin(ip.theta + angle);
			IntPoint iphit = map.world2map(phit);

			Point pfree = ip;
			pfree.x += (r - map.getDelta() * freeDelta) * cos(ip.theta + angle);
			pfree.y += (r - map.getDelta() * freeDelta) * sin(ip.theta + angle);
			pfree = pfree - phit;
			IntPoint ipfree = map.world2map(pfree);
			
			bool found = false;
			Point bestMu(0., 0.);
			for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++) {
				for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++) {
					IntPoint pr = iphit + IntPoint(xx, yy);
					IntPoint pf = pr + ipfree;
					const PointAccumulator& cell = map.cell(pr);
					const PointAccumulator& fcell = map.cell(pf);
					if (((double)cell) > m_fullnessThreshold && ((double)fcell) < m_fullnessThreshold) {
						Point mu = phit - cell.mean();
						if (!found) {
							bestMu = mu;
							found = true;
						} else {
							bestMu = (mu*mu) < (bestMu*bestMu) ? mu : bestMu;
						}
					}
				} // yy
			} // xx
			if (found) { s += exp(-1./m_gaussianSigma*bestMu*bestMu); }
		} // for each sensor reading
		return s;
	}

	// qiao@2015.08.27: use scan-matching to compute score
	inline double ScanMatcher::score_ScanMatching(const ScanMatcherMap & map, const OrientedPoint& p, const Infrared* ir_readings) const
	{
		double s_ret = 0; // return value
		OrientedPoint ip = p; // pose of sensor related to robot
		ip.x += cos(p.theta)*m_laserPose.x - sin(p.theta)*m_laserPose.y;
		ip.y += sin(p.theta)*m_laserPose.x + cos(p.theta)*m_laserPose.y;
		ip.theta += m_laserPose.theta;
		IntPoint p0 = map.world2map(ip);

		for (size_t i = 0; i < NUM_IR; i++) {
			double d = ir_readings[i].m_Range;
			double d_max = ir_readings[i].m_MaxRange;
			double angle = ir_readings[i].m_pose.theta;

			Point phit = ip + Point(d * cos(ip.theta + angle), d * sin(ip.theta + angle));
			// find max reachable point
			Point pmax = ip + Point(d_max * cos(ip.theta + angle), d_max * sin(ip.theta + angle));
			IntPoint p1 = map.world2map(pmax); 

			GridLineTraversalLine line;
			line.points = m_linePoints;
			GridLineTraversal::gridLine(p0, p1, &line);

			// find phit index
			//size_t idx_phit = (size_t)((double)line.num_points * d / d_max);
			
			// find pobs 
			Point pobs = Point(0., 0.);
			bool found = false;
			size_t j;
			for (j = 0; j < line.num_points - 1; j++) {
				const PointAccumulator& cell = map.cell(line.points[j]);
				double occ = (double)cell;
				if (occ > m_fullnessThreshold) {
					found = true;
					pobs = cell.mean();
					break;
				}
			}
			if (found) { 
				double d_err = euclidianDist(phit, pobs);
				s_ret += exp(-1. * (d_err * d_err) / m_gaussianSigma); 
			} else if (j == line.num_points) { // no obstacle nor hit
				s_ret += 1.0;
			}
		} // for each sensor reading
		return s_ret;
	}

	inline unsigned int ScanMatcher::likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) const {
		using namespace std;
		l = 0;
		s = 0;
		const double * angle = m_laserAngles + m_initialBeamsSkip;
		OrientedPoint lp = p;
		lp.x += cos(p.theta)*m_laserPose.x - sin(p.theta)*m_laserPose.y;
		lp.y += sin(p.theta)*m_laserPose.x + cos(p.theta)*m_laserPose.y;
		lp.theta += m_laserPose.theta;
		double noHit = nullLikelihood / (m_likelihoodSigma);
		unsigned int skip = 0;
		unsigned int c = 0;
		double freeDelta = map.getDelta()*m_freeCellRatio;
		for (const double* r = readings + m_initialBeamsSkip; r<readings + m_laserBeams; r++, angle++) {
			skip++;
			skip = skip>m_likelihoodSkip ? 0 : skip;
			if (*r>m_usableRange) continue;
			if (skip) continue;
			Point phit = lp;
			phit.x += *r*cos(lp.theta + *angle);
			phit.y += *r*sin(lp.theta + *angle);
			IntPoint iphit = map.world2map(phit);
			Point pfree = lp;
			pfree.x += (*r - freeDelta)*cos(lp.theta + *angle);
			pfree.y += (*r - freeDelta)*sin(lp.theta + *angle);
			pfree = pfree - phit;
			IntPoint ipfree = map.world2map(pfree);
			bool found = false;
			Point bestMu(0., 0.);
			for (int xx = -m_kernelSize; xx <= m_kernelSize; xx++)
				for (int yy = -m_kernelSize; yy <= m_kernelSize; yy++) {
					IntPoint pr = iphit + IntPoint(xx, yy);
					IntPoint pf = pr + ipfree;
					//AccessibilityState s=map.storage().cellState(pr);
					//if (s&Inside && s&Allocated){
					const PointAccumulator& cell = map.cell(pr);
					const PointAccumulator& fcell = map.cell(pf);
					if (((double)cell)>m_fullnessThreshold && ((double)fcell)<m_fullnessThreshold) {
						Point mu = phit - cell.mean();
						if (!found) {
							bestMu = mu;
							found = true;
						}
						else
							bestMu = (mu*mu)<(bestMu*bestMu) ? mu : bestMu;
					}
					//}	
				}
			if (found) {
				s += exp(-1. / m_gaussianSigma*bestMu*bestMu);
				c++;
			}
			if (!skip) {
				double f = (-1. / m_likelihoodSigma)*(bestMu*bestMu);
				l += (found) ? f : noHit;
			}
		}
		return c;
	}

};

#endif