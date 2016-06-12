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
            UInt16[] outputImageData;

            foreach (string fn in openFileDialog1.FileNames)
            {
                using (var inputImage = Tiff.Open(fn, "r"))
                {
                    width = inputImage.GetField(TiffTag.IMAGEWIDTH)[0].ToInt();
                    height = inputImage.GetField(TiffTag.IMAGELENGTH)[0].ToInt();


                    int ns = inputImage.StripSize();

                    int bytePerPixel = 4;
                    byte[] inputImageData = new byte[width * height * bytePerPixel];
                    var offset = 0;
                    int nstrip = inputImage.NumberOfStrips();
                    for (int i = 0; i < nstrip; i++)
                    {
                        offset += inputImage.ReadEncodedStrip(i, inputImageData, offset, ns);
                    }

                    float[] refv = new float[2];
                    refv[0] = -1;
                    refv[1] = -1;
                    byte[] buf = new byte[8];
                    Buffer.BlockCopy(refv, 0, buf, 0, 8);



                    float maxv = -1;
                   outputImageData = new UInt16[inputImageData.Length / 4];

                    bmp = new Bitmap(width, height);

                    UInt16 val = UInt16.MaxValue;

                    for (var i = 0; i < outputImageData.Length; i++)
                    {
                        float myFloat = System.BitConverter.ToSingle(inputImageData, i * 4);

                        if (myFloat > UInt16.MaxValue)
                            val = 0;
                        else
                        {
                            val = (UInt16)(myFloat );

                        }

                        outputImageData[i] = val;
                    }
                    FileInfo fi = new FileInfo(fn);
                    string outn=fi.Name.Replace(fi.Extension,".pgm");

                    PGMSave.Save(outputImageData, outn);
                }
                Color c = Color.FromArgb(0, 0, 0);
                for (int i = 0; i < width; i++)
                    for (int j = 0; j < height; j++)
                    {
                        float v = outputImageData[i + j * 640];

                        if (v < 6)
                        {
                            int ci = (int)(255.0 * v / 6.0);
                            c = Color.FromArgb(ci, ci, ci);
                        }
                        else
                            c = Color.FromArgb(0, 0, 0);

                        bmp.SetPixel(i, j, c);

                    }


            }


            // Draw the Image
            pictureBox1.Image = bmp;
        }
        
    }
}
