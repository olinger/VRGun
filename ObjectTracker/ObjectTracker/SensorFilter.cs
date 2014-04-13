using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK;

namespace ObjectTracker
{
	public class FloatFilter
	{
		private float[] circBuff;
		private int index;
		private int count;

		public FloatFilter(int cap)
		{
			circBuff = new float[cap];
			index = 0;
		}

		public void Add(float elem)
		{
			circBuff[index++] = elem;
			if (index >= circBuff.Length)
				index = 0;

			if (count < circBuff.Length)
				count++;
		}

		public float Mean
		{
			get
			{
				float sum = 0f;

				for (int i = 0; i < count; i++)
					sum += circBuff[i];

				return sum / (float)count;
			}
		}
	}

	public class VectorFilter
	{
		private Vector3[] circBuff;
		private int index;
		private int count;

		public VectorFilter(int cap)
		{
			circBuff = new Vector3[cap];
			index = 0;
		}

		public void Add(Vector3 elem)
		{
			circBuff[index++] = elem;
			if (index >= circBuff.Length)
				index = 0;

			if (count < circBuff.Length)
				count++;
		}

		public Vector3 Sum
		{
			get
			{
				Vector3 sum = Vector3.Zero;

				for (int i = 0; i < count; i++)
					sum += circBuff[i];

				return sum;
			}
		}

		public Vector3 Mean
		{
			get
			{
				Vector3 sum = Sum;
				return sum / (float)count;
			}
		}
	}
}
