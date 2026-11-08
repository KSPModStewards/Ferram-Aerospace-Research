using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

namespace FerramAerospaceResearch
{
    public static class Serialization
    {
        private delegate bool TryParseDelegate<T>(string str, out T value);

        private static readonly Dictionary<Type, Serializer> serializers = new Dictionary<Type, Serializer>
        {
            {
                typeof(int),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<int>(s, out value, int.TryParse),
                    Save = (node, s, value) => node.AddValue(s, (int)value)
                }
            },
            {
                typeof(float),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<float>(s, out value, float.TryParse),
                    Save = (node, s, value) => node.AddValue(s, (float)value)
                }
            },
            {
                typeof(double),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<double>(s, out value, double.TryParse),
                    Save = (node, s, value) => node.AddValue(s, (double)value)
                }
            },
            {
                typeof(long),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<long>(s, out value, long.TryParse),
                    Save = (node, s, value) => node.AddValue(s, (long)value)
                }
            },
            {
                typeof(string), new Serializer
                {
                    Parse = (string s, out object value) =>
                    {
                        value = s;
                        return true;
                    },
                    Save = (node, s, value) => node.AddValue(s, (string)value)
                }
            },
            {
                typeof(Color),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<Color>(s, out value, TryParseColor),
                    Save = (node, s, value) => node.AddValue(s, (Color)value)
                }
            },
            {
                typeof(Color32),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<Color32>(s, out value, TryParseColor),
                    Save = (node, s, value) => node.AddValue(s, (Color32)value)
                }
            },
            {
                typeof(Guid),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<Guid>(s, out value, TryParseGuid),
                    Save = (node, s, value) => node.AddValue(s, value)
                }
            },
            {
                typeof(Quaternion),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<Quaternion>(s, out value, TryParseQuaternion),
                    Save = (node, s, value) => node.AddValue(s, (Quaternion)value)
                }
            },
            {
                typeof(Rect),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<Rect>(s, out value, TryParseRect),
                    // as TryParseRect reads it back ("x,y,w,h"): AddValue(object) wrote Rect.ToString(), which it cannot
                    Save = (node, s, value) => node.AddValue(s, FormatRect((Rect)value))
                }
            },
            {
                typeof(Vector2),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<Vector2>(s, out value, TryParseVector2),
                    Save = (node, s, value) => node.AddValue(s, (Vector2)value)
                }
            },
            {
                typeof(Vector3),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<Vector3>(s, out value, TryParseVector3),
                    Save = (node, s, value) => node.AddValue(s, (Vector3)value)
                }
            },
            {
                typeof(bool),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<bool>(s, out value, bool.TryParse),
                    Save = (node, s, value) => node.AddValue(s, (bool)value)
                }
            },
            {
                typeof(Vector4),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<Vector4>(s, out value, TryParseVector4),
                    Save = (node, s, value) => node.AddValue(s, (Vector4)value)
                }
            },
            {
                typeof(uint),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<uint>(s, out value, uint.TryParse),
                    Save = (node, s, value) => node.AddValue(s, (uint)value)
                }
            },
            {
                typeof(ulong),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<ulong>(s, out value, ulong.TryParse),
                    Save = (node, s, value) => node.AddValue(s, (ulong)value)
                }
            },
            {
                typeof(string[]),
                new Serializer
                {
                    Parse = (string s, out object value) => TryParse<string[]>(s, out value, TryParseStringArray),
                    // a comma list, as TryParseStringArray reads it back: AddValue(object) wrote "System.String[]"
                    Save = (node, s, value) => node.AddValue(s, string.Join(",", (string[])value))
                }
            }
        };

        public static bool TryParseColor(string str, out Color value)
        {
            return ParseExtensions.TryParseColor(str, out value) || ColorUtility.TryParseHtmlString(str, out value);
        }

        public static bool TryParseColor(string str, out Color32 value)
        {
            return ParseExtensions.TryParseColor32(str, out value);
        }

        public static bool TryParseGuid(string str, out Guid g)
        {
            try
            {
                g = new Guid(str);
                return true;
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "While trying to parse Guid");
                g = Guid.Empty;
                return false;
            }
        }

        public static bool TryParseQuaternion(string str, out Quaternion value)
        {
            return ParseExtensions.TryParseQuaternion(str, out value);
        }

        private static string FormatRect(Rect r) =>
            string.Format(CultureInfo.InvariantCulture, "{0:R},{1:R},{2:R},{3:R}", r.x, r.y, r.width, r.height);

        public static bool TryParseRect(string str, out Rect value)
        {
            return ParseExtensions.TryParseRect(str, out value);
        }

        public static bool TryParseVector2(string str, out Vector2 value)
        {
            return ParseExtensions.TryParseVector2(str, out value);
        }

        public static bool TryParseVector3(string str, out Vector3 value)
        {
            return ParseExtensions.TryParseVector3(str, out value);
        }

        public static bool TryParseVector4(string str, out Vector4 value)
        {
            return ParseExtensions.TryParseVector4(str, out value);
        }

        public static bool TryParseStringArray(string str, out string[] value)
        {
            value = ParseExtensions.ParseArray(str, Array.Empty<char>());
            return true;
        }

        public static bool TryParseEnum(string str, Type type, out Enum value)
        {
            value = default;
            try
            {
                value = Enum.Parse(type, str, true) as Enum;
                return true;
            }
            catch (Exception e)
            {
                FARLogger.Exception(e, "While parsing enum");
                return false;
            }
        }

        private static bool TryParse<T>(string str, out object value, TryParseDelegate<T> parseDelegate)
        {
            value = default;
            if (!parseDelegate.Invoke(str, out T v))
                return false;
            value = v;
            return true;
        }

        public static void AddValue(
            ConfigNode node,
            string id,
            object value,
            Type valueType,
            bool isPatch = false,
            int? index = null
        )
        {
            if (isPatch)
            {
                id = index != null ? $"@{id},{((int)index).ToString()}" : $"%{id}";
            }

            if (valueType.IsEnum)
            {
                node.AddValue(id, value);
                return;
            }

            if (serializers.TryGetValue(valueType, out Serializer io))
            {
                io.Save(node, id, value);
                return;
            }

            FARLogger.DebugFormat("Unknown value type {0}", valueType);
            node.AddValue(id, value);
        }

        public static bool TryGetValue(string str, out object value, Type valueType)
        {
            value = null;
            if (str == null)
                return false;
            if (valueType.IsEnum)
            {
                if (!TryParseEnum(str, valueType, out Enum e))
                    return false;
                value = e;
                return true;
            }

            if (serializers.TryGetValue(valueType, out Serializer io))
            {
                if (!io.Parse(str, out object o))
                    return false;
                value = o;
                return true;
            }

            FARLogger.DebugFormat("Unknown value type {0}", valueType);
            return false;
        }

        public static bool TryGetValue(ConfigNode node, string id, out object value, Type valueType)
        {
            string str = node.GetValue(id);
            return TryGetValue(str, out value, valueType);
        }

        public static void MakeTopNode(
            ConfigNode child,
            string id,
            string name = null,
            bool isPatch = false
        )
        {
            string nodeName;
            if (isPatch)
            {
                nodeName = $"@{id}";
                if (name != null)
                    nodeName += $"[{name}]";
                nodeName += ":FOR[FerramAerospaceResearch]";
            }
            else
            {
                if (name != null)
                    child.AddValue("name", name);
                nodeName = id;
            }

            child.name = nodeName;
        }

        public static void AddNode(
            ConfigNode node,
            ConfigNode child,
            string id,
            string name = null,
            bool isPatch = false,
            int? index = null
        )
        {
            string nodeName;
            if (isPatch)
            {
                nodeName = $"@{id}";
                if (name != null)
                    nodeName += $"[{name}]";
                if (index != null)
                    nodeName += $",{((int)index).ToString()}";
            }
            else
            {
                if (name != null && !child.HasValue("name"))
                    child.AddValue("name", name);
                if (index != null && !child.HasValue("index"))
                    child.AddValue("index", (int)index);
                nodeName = id;
            }

            node.AddNode(nodeName, child);
        }

        private class Serializer
        {
            public delegate bool TryParseDelegate(string str, out object value);

            public delegate void SaveDelegate(ConfigNode node, string name, object value);

            public TryParseDelegate Parse;
            public SaveDelegate Save;
        }
    }
}
