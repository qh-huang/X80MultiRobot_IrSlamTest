#include <cstring>
#include <limits>
#include <list>
#include <iostream>
#include <conio.h>

#include <gmapping/scanmatcher/scanmatcher.h>
#include "gridlinetraversal.h"

//#define GENERATE_MAPS
const double CONST_PI = 3.141592653589793238463;

#undef max

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))
#define NUM_IR 6
//#define NUM_IR 1

namespace GMapping {

	using namespace std;

	const double ScanMatcher::nullLikelihood = -.5;

	ScanMatcher::ScanMatcher() : m_laserPose(0, 0, 0) {
		//m_laserAngles=0;
		m_laserBeams = 0;
		m_optRecursiveIterations = 3;
		m_activeAreaComputed = false;

		// This  are the dafault settings for a grid map of 5 cm
		m_llsamplerange = 0.01;
		m_llsamplestep = 0.01;
		m_lasamplerange = 0.005;
		m_lasamplestep = 0.005;
		m_enlargeStep = 10.;
		m_fullnessThreshold = 0.1;
		m_angularOdometryReliability = 0.;
		m_linearOdometryReliability = 0.;
		m_freeCellRatio = sqrt(2.);
		m_initialBeamsSkip = 0;

		/*
		// This  are the dafault settings for a grid map of 10 cm
		m_llsamplerange=0.1;
		m_llsamplestep=0.1;
		m_lasamplerange=0.02;
		m_lasamplestep=0.01;
		*/
		// This  are the dafault settings for a grid map of 20/25 cm
		/*
		m_llsamplerange=0.2;
		m_llsamplestep=0.1;
		m_lasamplerange=0.02;
		m_lasamplestep=0.01;
		m_generateMap=false;
		*/

		m_linePoints = new IntPoint[20000];
	}

	ScanMatcher::~ScanMatcher() {
		delete[] m_linePoints;
	}

	void ScanMatcher::invalidateActiveArea() {
		m_activeAreaComputed = false;
	}

	/*
	void ScanMatcher::computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const double* readings){
	if (m_activeAreaComputed)
	return;
	HierarchicalArray2D<PointAccumulator>::PointSet activeArea;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	IntPoint p0=map.world2map(lp);
	const double * angle=m_laserAngles;
	for (const double* r=readings; r<readings+m_laserBeams; r++, angle++)
	if (m_generateMap){
	double d=*r;
	if (d>m_laserMaxRange)
	continue;
	if (d>m_usableRange)
	d=m_usableRange;

	Point phit=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
	IntPoint p1=map.world2map(phit);

	d+=map.getDelta();
	//Point phit2=lp+Point(d*cos(lp.theta+*angle),d*sin(lp.theta+*angle));
	//IntPoint p2=map.world2map(phit2);
	IntPoint linePoints[20000] ;
	GridLineTraversalLine line;
	line.points=linePoints;
	//GridLineTraversal::gridLine(p0, p2, &line);
	GridLineTraversal::gridLine(p0, p1, &line);
	for (int i=0; i<line.num_points-1; i++){
	activeArea.insert(map.storage().patchIndexes(linePoints[i]));
	}
	if (d<=m_usableRange){
	activeArea.insert(map.storage().patchIndexes(p1));
	//activeArea.insert(map.storage().patchIndexes(p2));
	}
	} else {
	if (*r>m_laserMaxRange||*r>m_usableRange) continue;
	Point phit=lp;
	phit.x+=*r*cos(lp.theta+*angle);
	phit.y+=*r*sin(lp.theta+*angle);
	IntPoint p1=map.world2map(phit);
	assert(p1.x>=0 && p1.y>=0);
	IntPoint cp=map.storage().patchIndexes(p1);
	assert(cp.x>=0 && cp.y>=0);
	activeArea.insert(cp);

	}
	//this allocates the unallocated cells in the active area of the map
	//cout << "activeArea::size() " << activeArea.size() << endl;
	map.storage().setActiveArea(activeArea, true);
	m_activeAreaComputed=true;
	}
	*/
	void ScanMatcher::computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const double* readings) {
		if (m_activeAreaComputed)
			return;
		OrientedPoint lp = p;
		lp.x += cos(p.theta)*m_laserPose.x - sin(p.theta)*m_laserPose.y;
		lp.y += sin(p.theta)*m_laserPose.x + cos(p.theta)*m_laserPose.y;
		lp.theta += m_laserPose.theta;
		IntPoint p0 = map.world2map(lp);

		Point pmin(map.map2world(0, 0));
		Point pmax(map.map2world(map.getMapSizeX() - 1, map.getMapSizeY() - 1));

		if (lp.x<pmin.x) pmin.x = lp.x;
		if (lp.y<pmin.y) pmin.y = lp.y;
		if (lp.x>pmax.x) pmax.x = lp.x;
		if (lp.y>pmax.y) pmax.y = lp.y;

		/*determine the size of the area*/
		const double * angle = m_laserAngles + m_initialBeamsSkip;
		for (const double* r = readings + m_initialBeamsSkip; r<readings + m_laserBeams; r++, angle++) {
			if (*r>m_laserMaxRange || *r == 0.0 || isnan(*r)) continue;
			double d = *r>m_usableRange ? m_usableRange : *r;
			Point phit = lp;
			phit.x += d*cos(lp.theta + *angle);
			phit.y += d*sin(lp.theta + *angle);
			if (phit.x<pmin.x) pmin.x = phit.x;
			if (phit.y<pmin.y) pmin.y = phit.y;
			if (phit.x>pmax.x) pmax.x = phit.x;
			if (phit.y>pmax.y) pmax.y = phit.y;
		}
		//min=min-Point(map.getDelta(),map.getDelta());
		//max=max+Point(map.getDelta(),map.getDelta());

		if (!map.isInside(pmin) || !map.isInside(pmax)) {
			Point lmin(map.map2world(0, 0));
			Point lmax(map.map2world(map.getMapSizeX() - 1, map.getMapSizeY() - 1));
			//cerr << "CURRENT MAP " << lmin.x << " " << lmin.y << " " << lmax.x << " " << lmax.y << endl;
			//cerr << "BOUNDARY OVERRIDE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
			pmin.x = (pmin.x >= lmin.x) ? lmin.x : pmin.x - m_enlargeStep;
			pmax.x = (pmax.x <= lmax.x) ? lmax.x : pmax.x + m_enlargeStep;
			pmin.y = (pmin.y >= lmin.y) ? lmin.y : pmin.y - m_enlargeStep;
			pmax.y = (pmax.y <= lmax.y) ? lmax.y : pmax.y + m_enlargeStep;
			map.resize(pmin.x, pmin.y, pmax.x, pmax.y);
			//cerr << "RESIZE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
		}

		HierarchicalArray2D<PointAccumulator>::PointSet activeArea;
		/*allocate the active area*/
		angle = m_laserAngles + m_initialBeamsSkip;
		for (const double* r = readings + m_initialBeamsSkip; r<readings + m_laserBeams; r++, angle++)
			if (m_generateMap) {
				double d = *r;
				if (d>m_laserMaxRange || d == 0.0 || isnan(d))
					continue;
				if (d>m_usableRange)
					d = m_usableRange;
				Point phit = lp + Point(d*cos(lp.theta + *angle), d*sin(lp.theta + *angle));
				IntPoint p0 = map.world2map(lp);
				IntPoint p1 = map.world2map(phit);

				//IntPoint linePoints[20000] ;
				GridLineTraversalLine line;
				line.points = m_linePoints;
				GridLineTraversal::gridLine(p0, p1, &line);
				for (int i = 0; i<line.num_points - 1; i++) {
					assert(map.isInside(m_linePoints[i]));
					activeArea.insert(map.storage().patchIndexes(m_linePoints[i]));
					assert(m_linePoints[i].x >= 0 && m_linePoints[i].y >= 0);
				}
				if (d<m_usableRange) {
					IntPoint cp = map.storage().patchIndexes(p1);
					assert(cp.x >= 0 && cp.y >= 0);
					activeArea.insert(cp);
				}
			}
			else {
				if (*r>m_laserMaxRange || *r>m_usableRange || *r == 0.0 || isnan(*r)) continue;
				Point phit = lp;
				phit.x += *r*cos(lp.theta + *angle);
				phit.y += *r*sin(lp.theta + *angle);
				IntPoint p1 = map.world2map(phit);
				assert(p1.x >= 0 && p1.y >= 0);
				IntPoint cp = map.storage().patchIndexes(p1);
				assert(cp.x >= 0 && cp.y >= 0);
				activeArea.insert(cp);
			}

			//this allocates the unallocated cells in the active area of the map
			//cout << "activeArea::size() " << activeArea.size() << endl;
			/*
			cerr << "ActiveArea=";
			for (HierarchicalArray2D<PointAccumulator>::PointSet::const_iterator it=activeArea.begin(); it!= activeArea.end(); it++){
			cerr << "(" << it->x <<"," << it->y << ") ";
			}
			cerr << endl;
			*/
			map.storage().setActiveArea(activeArea, true);
			m_activeAreaComputed = true;
	}

	// qiao@2015.08.24
	void ScanMatcher::computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const Infrared* ir_readings) 
	{
		if (m_activeAreaComputed) return;

		OrientedPoint ip = p;
		ip.x += cos(p.theta) * m_laserPose.x - sin(p.theta) * m_laserPose.y;
		ip.y += sin(p.theta) * m_laserPose.y + cos(p.theta) * m_laserPose.y;
		ip.theta += m_laserPose.theta;
		IntPoint p0 = map.world2map(ip);

		Point min(map.map2world(0,0));
		Point max(map.map2world(map.getMapSizeX() - 1, map.getMapSizeY() - 1));

		if (ip.x < min.x) min.x = ip.x;
		if (ip.y < min.y) min.y = ip.y;
		if (ip.x > max.x) max.x = ip.x;
		if (ip.y > max.y) max.y = ip.y;

		// determine the size of the area
		//for (size_t i = 0; i < NELEMS(ir_readings); i++) {
		for (size_t i = 0; i < NUM_IR; i++) {
			double angle = ir_readings[i].m_pose.theta;
			double d = (ir_readings[i].m_Range > m_usableRange) ? m_usableRange : ir_readings[i].m_Range;
			Point phit = ip;
			phit.x += d * cos(ip.theta + angle);
			phit.y += d * sin(ip.theta + angle);

			if (phit.x < min.x) min.x = phit.x;
			if (phit.y < min.y) min.y = phit.y;
			if (phit.x > max.x) max.x = phit.x;
			if (phit.y > max.y) max.y = phit.y;
		}

		if (!map.isInside(min) || !map.isInside(max)) {
			// debug
			//_cprintf("ScanMatcher::computeActiveArea:\n");
			//_cprintf("ip=(%lf,%lf)\n", ip.x, ip.y);
			//for (size_t i = 0; i < NELEMS(ir_readings); i++) {
			for (size_t i = 0; i < NUM_IR; i++) {
				double angle = ir_readings[i].m_pose.theta;
				double d = (ir_readings[i].m_Range > m_usableRange) ? m_usableRange : ir_readings[i].m_Range;
				Point phit = ip;
				phit.x += d * cos(ip.theta + angle);
				phit.y += d * sin(ip.theta + angle);
				//_cprintf("phit[%d]=(%lf,%lf)\n", i, phit.x, phit.y);
			}
			//_cprintf("min=(%lf,%lf), max=(%lf,%lf)\n", min.x, min.y, max.x, max.y);
			//system("pause");

			Point lmin(map.map2world(0, 0));
			Point lmax(map.map2world(map.getMapSizeX() - 1, map.getMapSizeY() - 1));
			//cerr << "CURRENT MAP " << lmin.x << " " << lmin.y << " " << lmax.x << " " << lmax.y << endl;
			//cerr << "BOUNDARY OVERRIDE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
			min.x = (min.x >= lmin.x) ? lmin.x : min.x - m_enlargeStep;
			max.x = (max.x <= lmax.x) ? lmax.x : max.x + m_enlargeStep;
			min.y = (min.y >= lmin.y) ? lmin.y : min.y - m_enlargeStep;
			max.y = (max.y <= lmax.y) ? lmax.y : max.y + m_enlargeStep;
			map.resize(min.x, min.y, max.x, max.y);
			//cerr << "RESIZE " << min.x << " " << min.y << " " << max.x << " " << max.y << endl;
		}

		HierarchicalArray2D<PointAccumulator>::PointSet activeArea;
		// allocate the active area
		//for (size_t i = 0; i < NELEMS(ir_readings); i++) {
		for (size_t i = 0; i < NUM_IR; i++) {
			if (m_generateMap) {
				double d = ir_readings[i].m_Range;
				double angle = ir_readings[i].m_pose.theta;
				if (d > m_usableRange) { d = m_usableRange; }
				Point phit = ip + Point(d * cos(ip.theta + angle), d * sin(ip.theta + angle));
				
				IntPoint p0 = map.world2map(ip);
				IntPoint p1 = map.world2map(phit);
				GridLineTraversalLine line;
				line.points = m_linePoints;
				GridLineTraversal::gridLine(p0, p1, &line);
				for (size_t j = 0; j < line.num_points - 1; j++) {
					assert(map.isInside(m_linePoints[j]));
					activeArea.insert(map.storage().patchIndexes(m_linePoints[j]));
					assert(m_linePoints[j].x >= 0 && m_linePoints[j].y >= 0);
				}
				if (d<m_usableRange) {
					IntPoint cp = map.storage().patchIndexes(p1);
					assert(cp.x >= 0 && cp.y >= 0);
					activeArea.insert(cp);
					
				}
			} else {
				double d = ir_readings[i].m_Range;
				double angle = ir_readings[i].m_pose.theta;

				Point phit = ip;
				phit.x += d * cos(ip.theta + angle);
				phit.y += d * sin(ip.theta + angle);
				IntPoint p1 = map.world2map(phit);
				assert(p1.x >= 0 && p1.y >= 0);
				IntPoint cp = map.storage().patchIndexes(p1);
				assert(cp.x >= 0 && cp.y >= 0);
				activeArea.insert(cp);
			}
		} // for each element
		map.storage().setActiveArea(activeArea, true);
		m_activeAreaComputed = true;
	}

	double ScanMatcher::registerScan(ScanMatcherMap& map, const OrientedPoint& p, const double* readings) {
		if (!m_activeAreaComputed)
			computeActiveArea(map, p, readings);

		//this operation replicates the cells that will be changed in the registration operation
		map.storage().allocActiveArea();

		OrientedPoint lp = p;
		lp.x += cos(p.theta)*m_laserPose.x - sin(p.theta)*m_laserPose.y;
		lp.y += sin(p.theta)*m_laserPose.x + cos(p.theta)*m_laserPose.y;
		lp.theta += m_laserPose.theta;
		IntPoint p0 = map.world2map(lp);


		const double * angle = m_laserAngles + m_initialBeamsSkip;
		double esum = 0;
		for (const double* r = readings + m_initialBeamsSkip; r<readings + m_laserBeams; r++, angle++)
			if (m_generateMap) {
				double d = *r;
				if (d>m_laserMaxRange || d == 0.0 || isnan(d))
					continue;
				if (d>m_usableRange)
					d = m_usableRange;
				Point phit = lp + Point(d*cos(lp.theta + *angle), d*sin(lp.theta + *angle));
				IntPoint p1 = map.world2map(phit);
				//IntPoint linePoints[20000] ;
				GridLineTraversalLine line;
				line.points = m_linePoints;
				GridLineTraversal::gridLine(p0, p1, &line);
				for (int i = 0; i<line.num_points - 1; i++) {
					PointAccumulator& cell = map.cell(line.points[i]);
					double e = -cell.entropy();
					cell.update(false, Point(0, 0));
					e += cell.entropy();
					esum += e;
				}
				if (d<m_usableRange) {
					double e = -map.cell(p1).entropy();
					map.cell(p1).update(true, phit);
					e += map.cell(p1).entropy();
					esum += e;
				}
			}
			else {
				if (*r>m_laserMaxRange || *r>m_usableRange || *r == 0.0 || isnan(*r)) continue;
				Point phit = lp;
				phit.x += *r*cos(lp.theta + *angle);
				phit.y += *r*sin(lp.theta + *angle);
				IntPoint p1 = map.world2map(phit);
				assert(p1.x >= 0 && p1.y >= 0);
				map.cell(p1).update(true, phit);
			}
			//cout  << "informationGain=" << -esum << endl;
			return esum;
	}

	double ScanMatcher::registerScan(ScanMatcherMap & map, const OrientedPoint & p, const Infrared * ir_readings)
	{
		if (!m_activeAreaComputed) { computeActiveArea(map, p, ir_readings); }

		// this operation replicates the cells that will be changed in the registration operation
		map.storage().allocActiveArea();

		OrientedPoint ip = p;
		ip.x += cos(p.theta)*m_laserPose.x - sin(p.theta)*m_laserPose.y;
		ip.y += sin(p.theta)*m_laserPose.x + cos(p.theta)*m_laserPose.y;
		ip.theta += m_laserPose.theta;
		IntPoint p0 = map.world2map(ip);
		// debug
		//_cprintf("ScanMatcher::registerScan:\n");
		//_cprintf("ip(x,y,theta)=(%lf,%lf,%lf)\n", ip.x, ip.y, ip.theta * 180 / CONST_PI);
		//_cprintf("p0=world2map(ip)=(%d,%d)\n", p0.x, p0.y);
		//system("pause");

		double esum = 0;
		//for (const double* r = readings + m_initialBeamsSkip; r<readings + m_laserBeams; r++, angle++)
		//for (size_t i = 0; i < NELEMS(ir_readings); i++) {
		for (size_t i = 0; i < NUM_IR; i++) {
			double d = ir_readings[i].m_Range;
			double angle = ir_readings[i].m_pose.theta;
			//_cprintf("d=%lf, angle=%lf\n", d, angle * 180 / CONST_PI);
			if (m_generateMap) {
				//if (d > m_laserMaxRange || d == 0.0 || isnan(d)) continue;
				
				//if (d > m_usableRange) { d = m_usableRange; }
				//_cprintf("cos(ip.theta + angle)=%lf, sin(ip.theta + angle)=%lf\n", cos(ip.theta + angle), sin(ip.theta + angle));
				Point phit = ip + Point(d * cos(ip.theta + angle), d * sin(ip.theta + angle));
				//Point phit = Point(ip.x + d*cos(ip.theta + angle), ip.y + d*sin(ip.theta + angle));
				IntPoint p1 = map.world2map(phit);
				// debug
				//_cprintf("phit(x,y)=(%lf,%lf)\n", phit.x, phit.y);
				//_cprintf("p1=world2map(phit)=(%d,%d)\n", p1.x, p1.y);
				//system("pause");

				GridLineTraversalLine line;
				line.points = m_linePoints;
				GridLineTraversal::gridLine(p0, p1, &line);
				
				for (int j = 0; j < line.num_points - 1; j++) {
					PointAccumulator& cell = map.cell(line.points[j]);
					double e = -cell.entropy();
					cell.update(false, Point(0, 0));
					e += cell.entropy();
					esum += e;
				}
				if (d < m_usableRange) {
					double e = -map.cell(p1).entropy();
					map.cell(p1).update(true, phit);
					e += map.cell(p1).entropy();
					esum += e;
				}
			} else {
				// if (d > m_laserMaxRange || d > m_usableRange || d == 0.0 || isnan(d)) continue;
				Point phit = ip;
				phit.x += d * cos(ip.theta + angle);
				phit.y += d * sin(ip.theta + angle);
				IntPoint p1 = map.world2map(phit);
				assert(p1.x >= 0 && p1.y >= 0);
				map.cell(p1).update(true, phit);
			}
			//cout  << "informationGain=" << -esum << endl;
		} // for each ir ray
		return esum;
	}

	double ScanMatcher::icpOptimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings) const {
		double currentScore;
		double sc = score(map, init, readings);;
		OrientedPoint start = init;
		pnew = init;
		int iterations = 0;
		do {
			currentScore = sc;
			sc = icpStep(pnew, map, start, readings);
			//cerr << "pstart=" << start.x << " " <<start.y << " " << start.theta << endl;
			//cerr << "pret=" << pnew.x << " " <<pnew.y << " " << pnew.theta << endl;
			start = pnew;
			iterations++;
		} while (sc>currentScore);
		cerr << "i=" << iterations << endl;
		return currentScore;
	}

	double ScanMatcher::optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings) const {
		double bestScore = -1;
		OrientedPoint currentPose = init;
		double currentScore = score(map, currentPose, readings);
		double adelta = m_optAngularDelta, ldelta = m_optLinearDelta;
		unsigned int refinement = 0;
		enum Move { Front, Back, Left, Right, TurnLeft, TurnRight, Done };
		/*	cout << __PRETTY_FUNCTION__<<  " readings: ";
		for (int i=0; i<m_laserBeams; i++){
		cout << readings[i] << " ";
		}
		cout << endl;
		*/	int c_iterations = 0;
		do {
			if (bestScore >= currentScore) {
				refinement++;
				adelta *= .5;
				ldelta *= .5;
			}
			bestScore = currentScore;
			//		cout <<"score="<< currentScore << " refinement=" << refinement;
			//		cout <<  "pose=" << currentPose.x  << " " << currentPose.y << " " << currentPose.theta << endl;
			OrientedPoint bestLocalPose = currentPose;
			OrientedPoint localPose = currentPose;

			Move move = Front;
			do {
				localPose = currentPose;
				switch (move) {
				case Front:
					localPose.x += ldelta;
					move = Back;
					break;
				case Back:
					localPose.x -= ldelta;
					move = Left;
					break;
				case Left:
					localPose.y -= ldelta;
					move = Right;
					break;
				case Right:
					localPose.y += ldelta;
					move = TurnLeft;
					break;
				case TurnLeft:
					localPose.theta += adelta;
					move = TurnRight;
					break;
				case TurnRight:
					localPose.theta -= adelta;
					move = Done;
					break;
				default:;
				}

				double odo_gain = 1;
				if (m_angularOdometryReliability>0.) {
					double dth = init.theta - localPose.theta; 	dth = atan2(sin(dth), cos(dth)); 	dth *= dth;
					odo_gain *= exp(-m_angularOdometryReliability*dth);
				}
				if (m_linearOdometryReliability>0.) {
					double dx = init.x - localPose.x;
					double dy = init.y - localPose.y;
					double drho = dx*dx + dy*dy;
					odo_gain *= exp(-m_linearOdometryReliability*drho);
				}
				double localScore = odo_gain*score(map, localPose, readings);

				if (localScore>currentScore) {
					currentScore = localScore;
					bestLocalPose = localPose;
				}
				c_iterations++;
			} while (move != Done);
			currentPose = bestLocalPose;
			//		cout << "currentScore=" << currentScore<< endl;
			//here we look for the best move;
		} while (currentScore>bestScore || refinement<m_optRecursiveIterations);
		//cout << __PRETTY_FUNCTION__ << "bestScore=" << bestScore<< endl;
		//cout << __PRETTY_FUNCTION__ << "iterations=" << c_iterations<< endl;
		pnew = currentPose;
		return bestScore;
	}

	struct ScoredMove {
		OrientedPoint pose;
		double score;
		double likelihood;
	};

	typedef std::list<ScoredMove> ScoredMoveList;

	double ScanMatcher::optimize(OrientedPoint& _mean, ScanMatcher::CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& init, const double* readings) const {
		ScoredMoveList moveList;
		double bestScore = -1;
		OrientedPoint currentPose = init;
		ScoredMove sm = { currentPose,0,0 };
		unsigned int matched = likelihoodAndScore(sm.score, sm.likelihood, map, currentPose, readings);
		double currentScore = sm.score;
		moveList.push_back(sm);
		double adelta = m_optAngularDelta, ldelta = m_optLinearDelta;
		unsigned int refinement = 0;
		int count = 0;
		enum Move { Front, Back, Left, Right, TurnLeft, TurnRight, Done };
		do {
			if (bestScore >= currentScore) {
				refinement++;
				adelta *= .5;
				ldelta *= .5;
			}
			bestScore = currentScore;
			//		cout <<"score="<< currentScore << " refinement=" << refinement;
			//		cout <<  "pose=" << currentPose.x  << " " << currentPose.y << " " << currentPose.theta << endl;
			OrientedPoint bestLocalPose = currentPose;
			OrientedPoint localPose = currentPose;

			Move move = Front;
			do {
				localPose = currentPose;
				switch (move) {
				case Front:
					localPose.x += ldelta;
					move = Back;
					break;
				case Back:
					localPose.x -= ldelta;
					move = Left;
					break;
				case Left:
					localPose.y -= ldelta;
					move = Right;
					break;
				case Right:
					localPose.y += ldelta;
					move = TurnLeft;
					break;
				case TurnLeft:
					localPose.theta += adelta;
					move = TurnRight;
					break;
				case TurnRight:
					localPose.theta -= adelta;
					move = Done;
					break;
				default:;
				}
				double localScore, localLikelihood;

				double odo_gain = 1;
				if (m_angularOdometryReliability>0.) {
					double dth = init.theta - localPose.theta; 	dth = atan2(sin(dth), cos(dth)); 	dth *= dth;
					odo_gain *= exp(-m_angularOdometryReliability*dth);
				}
				if (m_linearOdometryReliability>0.) {
					double dx = init.x - localPose.x;
					double dy = init.y - localPose.y;
					double drho = dx*dx + dy*dy;
					odo_gain *= exp(-m_linearOdometryReliability*drho);
				}
				localScore = odo_gain*score(map, localPose, readings);
				//update the score
				count++;
				matched = likelihoodAndScore(localScore, localLikelihood, map, localPose, readings);
				if (localScore>currentScore) {
					currentScore = localScore;
					bestLocalPose = localPose;
				}
				sm.score = localScore;
				sm.likelihood = localLikelihood;//+log(odo_gain);
				sm.pose = localPose;
				moveList.push_back(sm);
				//update the move list
			} while (move != Done);
			currentPose = bestLocalPose;
			//cout << __PRETTY_FUNCTION__ << "currentScore=" << currentScore<< endl;
			//here we look for the best move;
		} while (currentScore>bestScore || refinement<m_optRecursiveIterations);
		//cout << __PRETTY_FUNCTION__ << "bestScore=" << bestScore<< endl;
		//cout << __PRETTY_FUNCTION__ << "iterations=" << count<< endl;

		//normalize the likelihood
		double lmin = 1e9;
		double lmax = -1e9;
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++) {
			lmin = it->likelihood<lmin ? it->likelihood : lmin;
			lmax = it->likelihood>lmax ? it->likelihood : lmax;
		}
		//cout << "lmin=" << lmin << " lmax=" << lmax<< endl;
		for (ScoredMoveList::iterator it = moveList.begin(); it != moveList.end(); it++) {
			it->likelihood = exp(it->likelihood - lmax);
			//cout << "l=" << it->likelihood << endl;
		}
		//compute the mean
		OrientedPoint mean(0, 0, 0);
		double lacc = 0;
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++) {
			mean = mean + it->pose*it->likelihood;
			lacc += it->likelihood;
		}
		mean = mean*(1. / lacc);
		//OrientedPoint delta=mean-currentPose;
		//cout << "delta.x=" << delta.x << " delta.y=" << delta.y << " delta.theta=" << delta.theta << endl;
		CovarianceMatrix cov = { 0.,0.,0.,0.,0.,0. };
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++) {
			OrientedPoint delta = it->pose - mean;
			delta.theta = atan2(sin(delta.theta), cos(delta.theta));
			cov.xx += delta.x*delta.x*it->likelihood;
			cov.yy += delta.y*delta.y*it->likelihood;
			cov.tt += delta.theta*delta.theta*it->likelihood;
			cov.xy += delta.x*delta.y*it->likelihood;
			cov.xt += delta.x*delta.theta*it->likelihood;
			cov.yt += delta.y*delta.theta*it->likelihood;
		}
		cov.xx /= lacc, cov.xy /= lacc, cov.xt /= lacc, cov.yy /= lacc, cov.yt /= lacc, cov.tt /= lacc;

		_mean = currentPose;
		_cov = cov;
		return bestScore;
	}

	double ScanMatcher::optimize(OrientedPoint &pnew, const ScanMatcherMap& map, const OrientedPoint& init, const Infrared * ir_readings) const
	{
		double bestScore = -1;
		OrientedPoint currentPose = init;
		//double currentScore = score(map, currentPose, ir_readings);
		double currentScore = score_ScanMatching(map, currentPose, ir_readings);
		double adelta = m_optAngularDelta, ldelta = m_optLinearDelta;
		unsigned int refinement = 0;
		enum Move { Front, Back, Left, Right, TurnLeft, TurnRight, Done };

		// debug
//		cout << __PRETTY_FUNCTION__<<  " readings: ";
		//for (int i = 0; i<NELEMS(ir_readings); i++) {
		for (int i = 0; i<NUM_IR; i++) {
				//cout << ir_readings[i].m_Range << " ";
			//_cprintf("ScanMatcher::optimize:ir_readings[%d]=%lf\n", i, ir_readings[i].m_Range);
		}

		int c_iterations = 0; // debug only
		
		do {
			if (bestScore >= currentScore) {
				refinement++;
				adelta *= 0.8;
				ldelta *= 0.8;
			}
			bestScore = currentScore;
			cout <<"score="<< currentScore << " refinement=" << refinement;
			cout <<"pose=" << currentPose.x  << " " << currentPose.y << " " << currentPose.theta << endl;
			OrientedPoint bestLocalPose = currentPose;
			OrientedPoint localPose = currentPose;

			Move move = Front;
			do {
				localPose = currentPose;
				switch (move) {
				case Front:
					localPose.x += ldelta;
					move = Back;
					break;
				case Back:
					localPose.x -= ldelta;
					move = Left;
					break;
				case Left:
					localPose.y -= ldelta;
					move = Right;
					break;
				case Right:
					localPose.y += ldelta;
					move = TurnLeft;
					break;
				case TurnLeft:
					localPose.theta += adelta;
					move = TurnRight;
					break;
				case TurnRight:
					localPose.theta -= adelta;
					move = Done;
					break;
				default:;
				}

				double odo_gain = 1;
				if (m_angularOdometryReliability > 0.) {
					double dth = init.theta - localPose.theta; 	
					dth = atan2(sin(dth), cos(dth)); 	
					dth *= dth;
					odo_gain *= exp(-m_angularOdometryReliability*dth);
				}
				if (m_linearOdometryReliability > 0.) {
					double dx = init.x - localPose.x;
					double dy = init.y - localPose.y;
					double drho = dx*dx + dy*dy;
					odo_gain *= exp(-m_linearOdometryReliability*drho);
				}
				double localScore = odo_gain*score(map, localPose, ir_readings);

				if (localScore>currentScore) {
					currentScore = localScore;
					bestLocalPose = localPose;
				}
				c_iterations++; // debug
			} while (move != Done);
			currentPose = bestLocalPose;
			cout << "currentScore=" << currentScore<< endl;
			//here we look for the best move;
		} while (currentScore>bestScore || refinement<m_optRecursiveIterations);
		
//		cout << __PRETTY_FUNCTION__ << "bestScore=" << bestScore<< endl;
//		cout << __PRETTY_FUNCTION__ << "iterations=" << c_iterations<< endl;
		
		pnew = currentPose;
		return bestScore;
	} 

	void ScanMatcher::setLaserParameters
		(unsigned int beams, double* angles, const OrientedPoint& lpose) {
		/*if (m_laserAngles)
		delete [] m_laserAngles;
		*/
		assert(beams<LASER_MAXBEAMS);
		m_laserPose = lpose;
		m_laserBeams = beams;
		//m_laserAngles=new double[beams];
		memcpy(m_laserAngles, angles, sizeof(double)*m_laserBeams);
	}


	double ScanMatcher::likelihood
		(double& _lmax, OrientedPoint& _mean, CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& p, const double* readings) {
		ScoredMoveList moveList;

		for (double xx = -m_llsamplerange; xx <= m_llsamplerange; xx += m_llsamplestep)
			for (double yy = -m_llsamplerange; yy <= m_llsamplerange; yy += m_llsamplestep)
				for (double tt = -m_lasamplerange; tt <= m_lasamplerange; tt += m_lasamplestep) {

					OrientedPoint rp = p;
					rp.x += xx;
					rp.y += yy;
					rp.theta += tt;

					ScoredMove sm;
					sm.pose = rp;

					likelihoodAndScore(sm.score, sm.likelihood, map, rp, readings);
					moveList.push_back(sm);
				}

		//OrientedPoint delta=mean-currentPose;
		//cout << "delta.x=" << delta.x << " delta.y=" << delta.y << " delta.theta=" << delta.theta << endl;
		//normalize the likelihood
		double lmax = -1e9;
		double lcum = 0;
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++) {
			lmax = it->likelihood>lmax ? it->likelihood : lmax;
		}
		for (ScoredMoveList::iterator it = moveList.begin(); it != moveList.end(); it++) {
			//it->likelihood=exp(it->likelihood-lmax);
			lcum += exp(it->likelihood - lmax);
			it->likelihood = exp(it->likelihood - lmax);
			//cout << "l=" << it->likelihood << endl;
		}

		OrientedPoint mean(0, 0, 0);
		double s = 0, c = 0;
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++) {
			mean = mean + it->pose*it->likelihood;
			s += it->likelihood*sin(it->pose.theta);
			c += it->likelihood*cos(it->pose.theta);
		}
		mean = mean*(1. / lcum);
		s /= lcum;
		c /= lcum;
		mean.theta = atan2(s, c);


		CovarianceMatrix cov = { 0.,0.,0.,0.,0.,0. };
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++) {
			OrientedPoint delta = it->pose - mean;
			delta.theta = atan2(sin(delta.theta), cos(delta.theta));
			cov.xx += delta.x*delta.x*it->likelihood;
			cov.yy += delta.y*delta.y*it->likelihood;
			cov.tt += delta.theta*delta.theta*it->likelihood;
			cov.xy += delta.x*delta.y*it->likelihood;
			cov.xt += delta.x*delta.theta*it->likelihood;
			cov.yt += delta.y*delta.theta*it->likelihood;
		}
		cov.xx /= lcum, cov.xy /= lcum, cov.xt /= lcum, cov.yy /= lcum, cov.yt /= lcum, cov.tt /= lcum;

		_mean = mean;
		_cov = cov;
		_lmax = lmax;
		return log(lcum) + lmax;
	}

	double ScanMatcher::likelihood
		(double& _lmax, OrientedPoint& _mean, CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& p,
			Gaussian3& odometry, const double* readings, double gain) {
		ScoredMoveList moveList;


		for (double xx = -m_llsamplerange; xx <= m_llsamplerange; xx += m_llsamplestep)
			for (double yy = -m_llsamplerange; yy <= m_llsamplerange; yy += m_llsamplestep)
				for (double tt = -m_lasamplerange; tt <= m_lasamplerange; tt += m_lasamplestep) {

					OrientedPoint rp = p;
					rp.x += xx;
					rp.y += yy;
					rp.theta += tt;

					ScoredMove sm;
					sm.pose = rp;

					likelihoodAndScore(sm.score, sm.likelihood, map, rp, readings);
					sm.likelihood += odometry.eval(rp) / gain;
					assert(!isnan(sm.likelihood));
					moveList.push_back(sm);
				}

		//OrientedPoint delta=mean-currentPose;
		//cout << "delta.x=" << delta.x << " delta.y=" << delta.y << " delta.theta=" << delta.theta << endl;
		//normalize the likelihood
		double lmax = -std::numeric_limits<double>::max();
		double lcum = 0;
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++) {
			lmax = it->likelihood>lmax ? it->likelihood : lmax;
		}
		for (ScoredMoveList::iterator it = moveList.begin(); it != moveList.end(); it++) {
			//it->likelihood=exp(it->likelihood-lmax);
			lcum += exp(it->likelihood - lmax);
			it->likelihood = exp(it->likelihood - lmax);
			//cout << "l=" << it->likelihood << endl;
		}

		OrientedPoint mean(0, 0, 0);
		double s = 0, c = 0;
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++) {
			mean = mean + it->pose*it->likelihood;
			s += it->likelihood*sin(it->pose.theta);
			c += it->likelihood*cos(it->pose.theta);
		}
		mean = mean*(1. / lcum);
		s /= lcum;
		c /= lcum;
		mean.theta = atan2(s, c);


		CovarianceMatrix cov = { 0.,0.,0.,0.,0.,0. };
		for (ScoredMoveList::const_iterator it = moveList.begin(); it != moveList.end(); it++) {
			OrientedPoint delta = it->pose - mean;
			delta.theta = atan2(sin(delta.theta), cos(delta.theta));
			cov.xx += delta.x*delta.x*it->likelihood;
			cov.yy += delta.y*delta.y*it->likelihood;
			cov.tt += delta.theta*delta.theta*it->likelihood;
			cov.xy += delta.x*delta.y*it->likelihood;
			cov.xt += delta.x*delta.theta*it->likelihood;
			cov.yt += delta.y*delta.theta*it->likelihood;
		}
		cov.xx /= lcum, cov.xy /= lcum, cov.xt /= lcum, cov.yy /= lcum, cov.yt /= lcum, cov.tt /= lcum;

		_mean = mean;
		_cov = cov;
		_lmax = lmax;
		double v = log(lcum) + lmax;
		assert(!isnan(v));
		return v;
	}

	void ScanMatcher::setMatchingParameters
		(double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations, double likelihoodSigma, unsigned int likelihoodSkip) {
		m_usableRange = urange;
		m_laserMaxRange = range;
		m_kernelSize = kernsize;
		m_optLinearDelta = lopt;
		m_optAngularDelta = aopt;
		m_optRecursiveIterations = iterations;
		m_gaussianSigma = sigma;
		m_likelihoodSigma = likelihoodSigma;
		m_likelihoodSkip = likelihoodSkip;
	}

};
