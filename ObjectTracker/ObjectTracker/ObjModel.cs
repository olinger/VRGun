using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using OpenTK;

namespace ObjectTracker
{
	public class ObjModel
	{
		private static readonly char[] lineSplitChars = { ' ' };

		private List<Vector3> verts;
		private List<Vector3> norms;

		public ObjModel(string path)
		{
			verts = new List<Vector3>();
			norms = new List<Vector3>();
			List<Vector3> tempVerts = new List<Vector3>();
			List<Vector3> tempNorms = new List<Vector3>();

			using (StreamReader reader = new StreamReader(path))
			{
				string file = reader.ReadToEnd();
				foreach (string l in file.Split('\n'))
				{
					//trim any extras
					string tl = l;
					int commentStart = l.IndexOf("#");
					if (commentStart != -1)
						tl = tl.Substring(0, commentStart);
					tl = tl.Trim();

					string[] line = tl.Split(lineSplitChars, StringSplitOptions.RemoveEmptyEntries);
					if (line == null || line.Length == 0)
						continue;

					switch (line[0])
					{
						case "v":
							if (line.Length < 4)
								continue;

							Vector3 v;
							if (!TryParseVec(line, 1, 2, 3, out v)) continue;
							tempVerts.Add(v);
							break;
						case "vn":
							if (line.Length < 4)
								continue;

							Vector3 n;
							if (!TryParseVec(line, 1, 2, 3, out n)) continue;
							tempNorms.Add(n);
							break;
						case "f":
							if (line.Length < 4)
								continue;
							else if (line.Length == 4)
							{
								int v0, v1, v2;
								int n0, n1, n2;
								if (!int.TryParse(line[1].Split('/')[0], out v0)) continue;
								if (!int.TryParse(line[2].Split('/')[0], out v1)) continue;
								if (!int.TryParse(line[3].Split('/')[0], out v2)) continue;
								if (!int.TryParse(line[1].Split('/')[2], out n0)) continue;
								if (!int.TryParse(line[2].Split('/')[2], out n1)) continue;
								if (!int.TryParse(line[3].Split('/')[2], out n2)) continue;

								v0 -= 1;
								v1 -= 1;
								v2 -= 1;
								n0 -= 1;
								n1 -= 1;
								n2 -= 1;

								verts.Add(tempVerts[v0]);
								verts.Add(tempVerts[v1]);
								verts.Add(tempVerts[v2]);
								norms.Add(tempNorms[n0]);
								norms.Add(tempNorms[n1]);
								norms.Add(tempNorms[n2]);
							}
							else
							{
								int v0, n0;
								if (!int.TryParse(line[1].Split('/')[0], out v0)) continue;
								if (!int.TryParse(line[1].Split('/')[2], out n0)) continue;

								v0 -= 1;
								n0 -= 1;

								for (int i = 2; i < line.Length - 1; i++)
								{
									int vi, vii;
									int ni, nii;
									if (!int.TryParse(line[i].Split('/')[0], out vi)) continue;
									if (!int.TryParse(line[i + 1].Split('/')[0], out vii)) continue;
									if (!int.TryParse(line[i].Split('/')[2], out ni)) continue;
									if (!int.TryParse(line[i + 1].Split('/')[2], out nii)) continue;

									vi -= 1;
									vii -= 1;
									ni -= 1;
									nii -= 1;

									verts.Add(tempVerts[v0]);
									verts.Add(tempVerts[vi]);
									verts.Add(tempVerts[vii]);
									norms.Add(tempNorms[n0]);
									norms.Add(tempNorms[ni]);
									norms.Add(tempNorms[nii]);
								}
							}
							break;
					}
				}
			}
		}

		public Vector3[] GetVertices()
		{
			return verts.ToArray();
		}

		public Vector3[] GetNormals()
		{
			return norms.ToArray();
		}

		private bool TryParseVec(string[] values, int x, int y, int z, out Vector3 v)
		{
			v = Vector3.Zero;

			if (!float.TryParse(values[x], NumberStyles.Any, CultureInfo.InvariantCulture, out v.X)) return false;
			if (!float.TryParse(values[y], NumberStyles.Any, CultureInfo.InvariantCulture, out v.Y)) return false;
			if (!float.TryParse(values[z], NumberStyles.Any, CultureInfo.InvariantCulture, out v.Z)) return false;

			return true;
		}
	}
}
