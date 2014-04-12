using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;

namespace location
{
    class Program
    {
        static void Main(string[] args)
        {
            string[] lines = File.ReadAllLines("imu_data.csv");
            string[][] data = new string[lines.Length][];
            string[] s1 = lines[0].Split(',');
            for(int i=1;i<lines.Length;i++)
            {
                string[] s = lines[i].Split(',');
                data[i-1] = s;
            }
            List<float> distance;
            List<float> velocity;
            List<float> acceleration;

            for(int i=0;i<data.Length)
            {
                for(int j=0;j<2;j++)
                {
                    acceleration.Add(data[i][j]);
                }
            }
            Console.Read();
        }
    }
}
