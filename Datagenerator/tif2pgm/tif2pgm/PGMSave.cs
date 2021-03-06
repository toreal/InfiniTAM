﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;

namespace tif2pgm
{
   public  class PGMSave
    {
        static public void Save(byte[] data, String fn)
        {
            FileStream sw = File.Open(fn, FileMode.Create);

            BinaryWriter bw = new BinaryWriter(sw);

            bw.Write(Encoding.UTF8.GetBytes("P6\n"));
            bw.Write(Encoding.UTF8.GetBytes("640 480\n"));
            bw.Write(Encoding.UTF8.GetBytes("255\n"));

            bw.Write(data);
            
            bw.Close();
            sw.Close();

        }



        static public void Save(short [] data,String fn)
        {
            FileStream sw = File.Open(fn,FileMode.Create);
            
            BinaryWriter bw = new BinaryWriter(sw);

            bw.Write(Encoding.UTF8.GetBytes("P5\n"));
            bw.Write(Encoding.UTF8.GetBytes("640 480\n"));
            bw.Write(Encoding.UTF8.GetBytes("65535\n"));

            foreach (short v in data)
            {
                byte[] byteArray = BitConverter.GetBytes(v);
                byte tmp = byteArray[0];
                byteArray[0] = byteArray[1];
                byteArray[1] = tmp;
                    
                bw.Write(byteArray);
            }
            bw.Close();
            sw.Close();

        }

    }
}
