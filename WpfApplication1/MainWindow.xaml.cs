using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace WpfApplication1
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    /// 

    using Microsoft.Kinect;
    using WpfApplication1;

    public partial class MainWindow : Window
    {

        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        IList<Body> _bodies;
        

        public MainWindow()
        {
            InitializeComponent();

            _sensor = connectActiveSensor();
            _reader = getReaderReady();
            
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            if (_reader != null)
            {
                _reader.Dispose();
            }

            if (_sensor != null)
            {
                _sensor.Close();
            }
        }

        public KinectSensor connectActiveSensor()
        {
            KinectSensor sensor = KinectSensor.GetDefault();

            if (sensor != null)
            {
                sensor.Open();
            }
            return sensor;
        }

        public MultiSourceFrameReader getReaderReady()
        {
         MultiSourceFrameReader reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color |
                                             FrameSourceTypes.Depth |
                                             FrameSourceTypes.Infrared |
                                             FrameSourceTypes.Body);
            reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
            return reader;
        }

        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            // Get a reference to the multi-frame
            var reference = e.FrameReference.AcquireFrame();

            // Open color frame
            using (var frame = reference.ColorFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    //camera.Source = ToBitmap(frame); //Display the camera

                }
            }

            // Open depth frame
            using (var frame = reference.DepthFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    // Do something with the frame...
                }
            }

            // Open infrared frame
            using (var frame = reference.InfraredFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    // Do something with the frame...
                }
            }

            using (var frame2 = reference.BodyFrameReference.AcquireFrame())
            {

                if (frame2 != null)
                {

                    canvas.Children.Clear();

                    _bodies = new Body[frame2.BodyFrameSource.BodyCount];

                    frame2.GetAndRefreshBodyData(_bodies);

                    foreach (var body in _bodies)
                    {
                        if (body != null)
                        {

                            if (body.IsTracked)
                            {
                                //Should be drawing here but ballec
                            }
                             DrawSkeleton(canvas, body);
                            /*foreach (Joint joint in body.Joints.Values)
                            {
                                DrawPoint(canvas, joint);
                            } */
                        }
                    }
                }
            }
        }



        private ImageSource ToBitmap(ColorFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;
            PixelFormat format = PixelFormats.Bgr32;


            byte[] pixels = new byte[width * height * ((PixelFormats.Bgr32.BitsPerPixel + 7) / 8)];

            if (frame.RawColorImageFormat == ColorImageFormat.Bgra)
            {
                frame.CopyRawFrameDataToArray(pixels);
            }
            else
            {
                frame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);
            }

            int stride = width * format.BitsPerPixel / 8;


            return BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);
        }

        public ImageSource ToBitmap(DepthFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;
            PixelFormat format = PixelFormats.Bgr32;

            ushort minDepth = frame.DepthMinReliableDistance;
            ushort maxDepth = frame.DepthMaxReliableDistance;

            ushort[] pixelData = new ushort[width * height];
            byte[] pixels = new byte[width * height * (format.BitsPerPixel + 7) / 8];

            frame.CopyFrameDataToArray(pixelData);

            int colorIndex = 0;
            for (int depthIndex = 0; depthIndex < pixelData.Length; ++depthIndex)
            {
                ushort depth = pixelData[depthIndex];

                byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);

                pixels[colorIndex++] = intensity; // Blue
                pixels[colorIndex++] = intensity; // Green
                pixels[colorIndex++] = intensity; // Red

                ++colorIndex;
            }

            int stride = width * format.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, format, null, pixels, stride);
        }

        public ImageSource ToBitmap(InfraredFrame frame)
        {
            int width = frame.FrameDescription.Width;
            int height = frame.FrameDescription.Height;
            PixelFormat format = PixelFormats.Bgr32;

            ushort[] frameData = new ushort[width * height];
            byte[] pixels = new byte[width * height * (format.BitsPerPixel + 7) / 8];

            frame.CopyFrameDataToArray(frameData);

            int colorIndex = 0;
            for (int infraredIndex = 0; infraredIndex < frameData.Length; infraredIndex++)
            {
                ushort ir = frameData[infraredIndex];

                byte intensity = (byte)(ir >> 7);

                pixels[colorIndex++] = (byte)(intensity / 1); // Blue
                pixels[colorIndex++] = (byte)(intensity / 1); // Green   
                pixels[colorIndex++] = (byte)(intensity / 0.4); // Red

                colorIndex++;
            }

            int stride = width * format.BitsPerPixel / 8;

            return BitmapSource.Create(width, height, 96, 96, format, null, pixels, stride);
        }



        #region Body

        public static Joint ScaleTo(Joint joint, double width, double height, float skeletonMaxX, float skeletonMaxY)
        {
            joint.Position = new CameraSpacePoint
            {
                X = Scale(width, skeletonMaxX, joint.Position.X),
                Y = Scale(height, skeletonMaxY, -joint.Position.Y),
                Z = joint.Position.Z
            };

            return joint;
        }

        public static Joint ScaleTo(Joint joint, double width, double height)
        {
            return ScaleTo(joint, width, height, 1.0f, 1.0f);
        }

        private static float Scale(double maxPixel, double maxSkeleton, float position)
        {
            float value = (float)((((maxPixel / maxSkeleton) / 2) * position) + (maxPixel / 2));

            if (value > maxPixel)
            {
                return (float)maxPixel;
            }

            if (value < 0)
            {
                return 0;
            }

            return value;
        }

        #endregion

        #region Drawing

        public static void DrawSkeleton(Canvas canvas, Body body)
        {
            if (body == null) return;

            foreach (Joint joint in body.Joints.Values)
            {
                DrawPoint(canvas,joint);
            }

            DrawLine(canvas,body.Joints[JointType.Head], body.Joints[JointType.Neck]);
            DrawLine(canvas,body.Joints[JointType.Neck], body.Joints[JointType.SpineShoulder]);
            DrawLine(canvas,body.Joints[JointType.SpineShoulder], body.Joints[JointType.ShoulderLeft]);
            DrawLine(canvas,body.Joints[JointType.SpineShoulder], body.Joints[JointType.ShoulderRight]);
            DrawLine(canvas,body.Joints[JointType.SpineShoulder], body.Joints[JointType.SpineMid]);
            DrawLine(canvas,body.Joints[JointType.ShoulderLeft], body.Joints[JointType.ElbowLeft]);
            DrawLine(canvas,body.Joints[JointType.ShoulderRight], body.Joints[JointType.ElbowRight]);
            DrawLine(canvas,body.Joints[JointType.ElbowLeft], body.Joints[JointType.WristLeft]);
            DrawLine(canvas,body.Joints[JointType.ElbowRight], body.Joints[JointType.WristRight]);
            DrawLine(canvas,body.Joints[JointType.WristLeft], body.Joints[JointType.HandLeft]);
            DrawLine(canvas,body.Joints[JointType.WristRight], body.Joints[JointType.HandRight]);
            DrawLine(canvas,body.Joints[JointType.HandLeft], body.Joints[JointType.HandTipLeft]);
            DrawLine(canvas,body.Joints[JointType.HandRight], body.Joints[JointType.HandTipRight]);
            DrawLine(canvas,body.Joints[JointType.HandTipLeft], body.Joints[JointType.ThumbLeft]);
            DrawLine(canvas,body.Joints[JointType.HandTipRight], body.Joints[JointType.ThumbRight]);
            DrawLine(canvas,body.Joints[JointType.SpineMid], body.Joints[JointType.SpineBase]);
            DrawLine(canvas,body.Joints[JointType.SpineBase], body.Joints[JointType.HipLeft]);
            DrawLine(canvas,body.Joints[JointType.SpineBase], body.Joints[JointType.HipRight]);
            DrawLine(canvas,body.Joints[JointType.HipLeft], body.Joints[JointType.KneeLeft]);
            DrawLine(canvas,body.Joints[JointType.HipRight], body.Joints[JointType.KneeRight]);
            DrawLine(canvas,body.Joints[JointType.KneeLeft], body.Joints[JointType.AnkleLeft]);
            DrawLine(canvas,body.Joints[JointType.KneeRight], body.Joints[JointType.AnkleRight]);
            DrawLine(canvas,body.Joints[JointType.AnkleLeft], body.Joints[JointType.FootLeft]);
            DrawLine(canvas,body.Joints[JointType.AnkleRight], body.Joints[JointType.FootRight]);
        }

        public static void DrawPoint(Canvas canvas, Joint joint)
        {
            if (joint.TrackingState == TrackingState.NotTracked) return;

            joint = ScaleTo(joint,canvas.ActualWidth, canvas.ActualHeight);
            double x = Math.Log(joint.Position.Z + 1);

            Ellipse ellipse = new Ellipse
            {
                Width = 15 /x*1.5,
                Height = 15 /x*1.5,
                Fill = new SolidColorBrush(Color.FromArgb(255, (byte)(x / 100), (byte)(x * 127.5), (byte)(x * 30)))
                //new SolidColorBrush(Colors.LightBlue)
            };
            Canvas.SetLeft(ellipse, joint.Position.X - ellipse.Width / 2);
            Canvas.SetTop(ellipse, joint.Position.Y - ellipse.Height / 2);

            canvas.Children.Add(ellipse);
        }

        public static void DrawLine(Canvas canvas, Joint first, Joint second)
        {
            if (first.TrackingState == TrackingState.NotTracked || second.TrackingState == TrackingState.NotTracked) return;

            first = ScaleTo(first,canvas.ActualWidth, canvas.ActualHeight);
            second = ScaleTo(second,canvas.ActualWidth, canvas.ActualHeight);

            Line line = new Line
            {
                X1 = first.Position.X,
                Y1 = first.Position.Y,
                X2 = second.Position.X,
                Y2 = second.Position.Y,
                StrokeThickness = 8,
                Stroke = new SolidColorBrush(Colors.LightBlue)
            };

            canvas.Children.Add(line);
        }

        #endregion

        private void lavoid() {
        }
    }
}
