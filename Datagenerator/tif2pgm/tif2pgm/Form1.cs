using BitMiracle.LibTiff.Classic;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Windows.Media.Imaging;

namespace tif2pgm
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            openFileDialog1.ShowDialog();
            Bitmap bmp = null;
            int width, height;
           
            foreach (string fn in openFileDialog1.FileNames)
            {
                using (var inputImage = Tiff.Open(fn, "r"))
                {
                    byte[] inputImageData;
                    width = inputImage.GetField(TiffTag.IMAGEWIDTH)[0].ToInt();
                    height = inputImage.GetField(TiffTag.IMAGELENGTH)[0].ToInt();
                    var offset = 0;
                    int nstrip = inputImage.NumberOfStrips();
                    int ns = inputImage.StripSize();
                    int bytePerPixel = 4;// 16;

                    inputImageData = new byte[width * height * bytePerPixel];
                    

                    for (int i = 0; i < nstrip; i++)
                    {
                        offset += inputImage.ReadEncodedStrip(i, inputImageData, offset, ns);
                    }


                    FileInfo fi = new FileInfo(fn);
                   
                    if (fi.Name.StartsWith("c"))
                    {
                        string outn = fi.Name.Replace(fi.Extension, ".ppm");
                        outn = outn.Substring(1);
                        if (outn.Length == 7)
                            outn = "0" + outn;
                        byte[] outdata = new byte[width*height*3];

                        for (int j = 0; j < height; j++)
                        for(int i = 0; i < width; i++)
                            {
                                outdata[(j * width + i) * 3 + 0] = inputImageData[(j * width + i) * bytePerPixel + 0];
                                outdata[(j * width + i) * 3 + 1] = inputImageData[(j * width + i) * bytePerPixel + 1];
                                outdata[(j * width + i) * 3 + 2] = inputImageData[(j * width + i) * bytePerPixel + 2];

                                //    int i1= inputImageData[(j * width + i) * bytePerPixel + 4];
                                //    int i2= inputImageData[(j * width + i) * bytePerPixel + 8];
                                //int i3= inputImageData[(j * width + i) * bytePerPixel + 12];
                                //outdata[(j * width + i) * 3 + 0] = (byte)i1;
                                //outdata[(j * width + i) * 3 + 1] = (byte)i2;
                                //outdata[(j * width + i) * 3 + 2] = (byte)i3;

                            }


                        PGMSave.Save(outdata, outn);


                    }
                    else
                    {
                        string outn = fi.Name.Replace(fi.Extension, ".pgm");

                        if (outn.Length == 7)
                            outn = "0" + outn;
                        string outn2 = "M" + outn;

                        float maxv = -1;
                        short[] outputImageData = new short[inputImageData.Length / 4];
                        byte[] outputImageData2 = new byte[width*height*3];

                        bmp = new Bitmap(width, height);

                        short val = short.MaxValue;
                        byte val2 = 0;

                        int bitof = offset / (width * height);

                        for (var i = 0; i < outputImageData.Length; i++)
                        {
                            float myFloat = System.BitConverter.ToSingle(inputImageData, i * bitof);

                            if (myFloat > short.MaxValue)
                            {
                                val = -1;
                                val2 = 0;
                            }
                            else
                            {
                                val = (short)(myFloat);
                                val2 = 255;
                            }

                            outputImageData[i] = val;

                            for(int k=0; k< 3;k++)
                            outputImageData2[i*3+k] = val2;
                        }
                        PGMSave.Save(outputImageData, outn);
                        PGMSave.Save(outputImageData2, outn2);
                    }
                }
                //Color c = Color.FromArgb(0, 0, 0);
                //for (int i = 0; i < width; i++)
                //    for (int j = 0; j < height; j++)
                //    {
                //        float v = outputImageData[i + j * 640];

                //        if (v < 6)
                //        {
                //            int ci = (int)(255.0 * v / 6.0);
                //            c = Color.FromArgb(ci, ci, ci);
                //        }
                //        else
                //            c = Color.FromArgb(0, 0, 0);

                //        bmp.SetPixel(i, j, c);

                //    }


            }


            // Draw the Image
            pictureBox1.Image = bmp;
        }
        
    }
}
