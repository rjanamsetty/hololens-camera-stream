using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using OpenCVForUnity.CoreModule;
using OpenCVForUnity.Features2dModule;
using OpenCVForUnity.ImgprocModule;
using UnityEngine;
using UnityEngine.Networking;

namespace Script
{
    public class SquareDetection: IDisposable
    {
        private readonly string filepath;
        private TickMeter _ticks = new TickMeter();
        private long _startTick;
        private long _referenceTick;
        private int _fullIterations;
        private int _shortIterations;
        private long _fullIterationsTicks;
        private long _shortIterationsTicks;

        public SquareDetection()
        {
            var timestamp = DateTime.Now.ToString("yyyy-MM-dd-HH-mm-ss");
            filepath = Application.persistentDataPath + "/" + timestamp + ".csv";
            _startTick = -_ticks.getTimeTicks();
        }

        /// <summary>
        ///     Finds a cosine of angle between vectors from center->first and from center->last
        /// </summary>
        /// <param name="first">First point</param>
        /// <param name="last">Last Point</param>
        /// <param name="center">Center Point</param>
        /// <returns></returns>
        private double Angle(Point first, Point last, Point center)
        {
            var dx1 = first.x - center.x;
            var dy1 = first.y - center.y;
            var dx2 = last.x - center.x;
            var dy2 = last.y - center.y;
            return (dx1 * dx2 + dy1 * dy2) / Math.Sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
        }

        /// <summary>
        /// Outputs data into a consistent record taking format
        /// </summary>
        /// <param name="label">Label to identify the data</param>
        /// <param name="value">Value to associate the data with</param>
        private void writeData(string label, double value)
        {
            using (var sw = new StreamWriter(filepath))
            {
                sw.WriteLine(label + "," + value);
            }
        }

        /// <summary>
        /// Records the time taken in ticks
        /// </summary>
        /// <param name="stage">The stage where the calculation is</param>
        /// <param name="endTick"></param>
        private void recordTimeTaken(string stage, long startTick)
        {
            var duration = _ticks.getTimeTicks() - startTick;
            writeData(stage + " Ticks", duration);
        }

        /// <summary>
        ///     Returns sequence of squares detected on the image.
        /// </summary>
        /// <param name="image">The mat to find squares in</param>
        /// <returns>A list of MatOfPoints referencing all squares</returns>
        public List<MatOfPoint> FindSquares(Mat image)
        {
            // Set the reference point for tick counting
            _referenceTick = _ticks.getTimeTicks();

            // Allocate the variables appropriately
            var gray = Extensions.BGRAtoGrayscale(image);
            var squares = new List<MatOfPoint>();
            var edges = new Mat();
            var filtered = new Mat(image.size(), CvType.CV_8U);
            var hierarchy = new Mat();

            // Blur all but the edges
            Imgproc.bilateralFilter(gray, filtered, 15, 75, 75);

            var contours = new List<MatOfPoint>();

            // find the edges of the image
            Imgproc.Canny(filtered, edges, 0, 50, 5);
            Imgproc.dilate(edges, edges, new Mat(), new Point(-1, -1));

            // find contours and store them all as a list
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            var approx = new MatOfPoint2f();

            // test each contour
            foreach (var contour in contours)
            {
                // approximate contour with accuracy proportional
                // to the contour perimeter
                var curve = new MatOfPoint2f();
                curve.fromArray(contour.toArray());
                var epsilon = Imgproc.arcLength(curve, true) * 0.02;
                Imgproc.approxPolyDP(curve, approx, epsilon, true);
                var approxList = approx.toList();

                // square contours should have 4 vertices after approximation
                // relatively large area (to filter out noisy contours)
                // and be convex.
                // Note: absolute value of an area is used because
                // area may be positive or negative - in accordance with the
                // contour orientation
                var result = new MatOfPoint();
                result.fromArray(approx.toArray());
                if (approxList.Count != 4 || Math.Abs(Imgproc.contourArea(approx)) < 1000 ||
                    !Imgproc.isContourConvex(result)) continue;
                squares.Add(result);
            }

            if (squares.Count > 0) _fullIterations++;
            else
            {
                _shortIterations++;
                _shortIterationsTicks += _ticks.getTimeTicks() - _referenceTick;
            }

            return squares;
        }

        private List<Point> OrderPoints(MatOfPoint square)
        {
            var moments = Imgproc.moments(square);
            var center = new Point(moments.m10 / moments.m00, moments.m01 / moments.m00);
            return square.toList().OrderBy(pt => Math.Atan2(pt.x - center.x, pt.y - center.y)).ToList();
        }

        private double FindContourDistance(MatOfPoint contour, Point reference)
        {
            var square = new MatOfPoint2f();
            square.fromArray(contour.toArray());
            return Imgproc.pointPolygonTest(new MatOfPoint2f(square), reference, true);
        }

        public List<MatOfPoint> SelectOptimalSquare(List<MatOfPoint> squares, Mat bgraMat, Point reference)
        {
            if (squares.Count == 0) return new List<MatOfPoint>();
            var validSquare = squares.FindAll(square => CalculateArea(OrderPoints(square)) < 0.9 * GetMatArea(bgraMat));
            if (validSquare.Count == 0) return new List<MatOfPoint>();
            if (!reference.Equals(Extensions.InvalidPoint()))
            {
                var ordered = validSquare.OrderBy(square => FindContourDistance(square, reference)).ToList();
                validSquare.FindAll(square => FindContourDistance(square, reference) >= 0);
                if (validSquare.Count == 0) return ordered;
            }

            var validSquares = squares.OrderByDescending(square => CalculateArea(OrderPoints(square))).ToList();
            return validSquares.Count == 0 ? new List<MatOfPoint>() : validSquares;
        }

        private double GetMatArea(Mat bgraMat)
        {
            var height = bgraMat.height();
            var width = bgraMat.width();
            var points = new List<Point>
                {new Point(0, 0), new Point(0, height), new Point(width, height), new Point(width, 0)};
            return CalculateArea(points);
        }

        private double CalculateArea(List<Point> points)
        {
            // Add the first point to the end.
            var numPoints = points.Count;
            var pts = new Point[numPoints + 1];
            points.CopyTo(pts, 0);
            pts[numPoints] = points[0];

            // Get the areas.
            var area = 0d;
            for (var i = 0; i < numPoints; i++)
                area +=
                    (pts[i + 1].x - pts[i].x) *
                    (pts[i + 1].y + pts[i].y) / 2;

            // Return the result.
            return Math.Abs(area);
        }

        public List<double> MeasurePoints(Mat bgraMat, MatOfPoint contour)
        {
            var boundingRect = Imgproc.boundingRect(contour);
            var roi = Extensions.BGRAtoGrayscale(new Mat(bgraMat, boundingRect));
            var results = new List<double> {MeasureContours(roi), MeasureFeatures(roi)};

            // Measure total time taken
            _fullIterations++;
            _fullIterationsTicks += _ticks.getTimeTicks() - _referenceTick;
            
            return results;
        }

        /// <summary>
        /// Performs features detection on region of interest
        /// </summary>
        /// <param name="roi">Region of interest to perform operations on</param>
        /// <returns>Count of features</returns>
        private static double MeasureFeatures(Mat roi)
        {
            var orb = ORB.create(10000);
            var matPoints = new MatOfKeyPoint();
            orb.detect(roi, matPoints);
            return matPoints.toList().Count;
        }

        /// <summary>
        /// Performs contour detection on region of intrest 
        /// </summary>
        /// <param name="roi">Region of interest to perform operations on</param>
        /// <returns>Count of contours</returns>
        private static double MeasureContours(Mat roi)
        {
            // Contour detection
            var edges = new Mat();
            var hierarchy = new Mat();
            var contours = new List<MatOfPoint>();
            Imgproc.Canny(roi, edges, 0, 50, 5);
            Imgproc.dilate(edges, edges, new Mat(), new Point(-1, -1));
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            var count = contours.Count;
            return count;
        }

        /// <summary>
        /// Disposes the object by saving any final data
        /// </summary>
        public void Dispose()
        {
            var totalTicks = _fullIterationsTicks + _shortIterationsTicks;
            var totalIterations = _fullIterations + _shortIterations;
            writeData("Total CV Ticks", totalTicks);
            writeData("Total CV Iterations", totalIterations);
            writeData("Average CV Ticks", totalTicks/totalIterations);
            writeData("Total Full CV Executions", _fullIterations);
            writeData("Average Full CV Execution Ticks", _fullIterationsTicks/_fullIterations);
            writeData("Total Partial CV Executions", _shortIterations);
            writeData("Average Partial CV Execution Ticks", _shortIterations/_shortIterationsTicks);
            _ticks?.Dispose();

        }
    }
}