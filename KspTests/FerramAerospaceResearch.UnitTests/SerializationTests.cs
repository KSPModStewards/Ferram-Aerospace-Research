using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using Xunit;

namespace FerramAerospaceResearch.UnitTests
{
    /// <summary>
    /// FAR's config value serializers (Serialization): every value type a [ConfigValue] can have is written to a
    /// ConfigNode and parsed back to the same value; a value that does not parse, an enum name that does not exist and a
    /// type no serializer knows fail without throwing; the node names FAR writes for configs and MM patches.
    /// </summary>
    [Collection(TestLog.Collection)]
    public class SerializationTests
    {
        private enum Fruit
        {
            Apple,
            Banana
        }

        public SerializationTests()
        {
            UnityRuntime.RequireLog();   // ConfigNode, Unity's value types and FAR's logger
        }

        public static IEnumerable<object[]> Values()
        {
            yield return new object[] { typeof(int), -42 };
            // stripped Unity assemblies throw on Color's constructor: one row, which the constructor's skip then skips
            if (!UnityRuntime.Available) yield break;
            yield return new object[] { typeof(float), 0.375f };
            yield return new object[] { typeof(double), -1234.0625 };
            yield return new object[] { typeof(long), 1L << 40 };
            yield return new object[] { typeof(bool), true };
            yield return new object[] { typeof(uint), 4000000000u };
            yield return new object[] { typeof(ulong), 1UL << 63 };
            yield return new object[] { typeof(string), "some text" };
            yield return new object[] { typeof(Color), new Color(0.25f, 0.5f, 0.75f, 1f) };
            yield return new object[] { typeof(Color32), new Color32(10, 20, 30, 255) };
            yield return new object[] { typeof(Guid), new Guid("0f8fad5b-d9cb-469f-a165-70867728950e") };
            yield return new object[] { typeof(Quaternion), new Quaternion(0.5f, -0.5f, 0.5f, 0.5f) };
            yield return new object[] { typeof(Vector2), new Vector2(1.5f, -2f) };
            yield return new object[] { typeof(Vector3), new Vector3(1.5f, -2f, 0.25f) };
            yield return new object[] { typeof(Vector4), new Vector4(1.5f, -2f, 0.25f, 8f) };
            yield return new object[] { typeof(Fruit), Fruit.Banana };
        }

        [SkippableTheory]
        [MemberData(nameof(Values))]
        public void AValueRoundTripsThroughAConfigNode(Type type, object value)
        {
            var node = new ConfigNode("TEST");
            Serialization.AddValue(node, "v", value, type);
            Assert.True(node.HasValue("v"), node.ToString());
            Assert.True(Serialization.TryGetValue(node, "v", out object back, type), $"{type.Name} '{node.GetValue("v")}'");
            Assert.IsType(type, back);
            Assert.Equal(value, back);
        }

        // Regression: the string[] serializer's Save called ConfigNode.AddValue(string, object) (ConfigNode has no
        // string[] overload), which wrote "System.String[]" instead of the items.
        [SkippableFact]
        public void AStringArrayRoundTripsAsACommaList()
        {
            var node = new ConfigNode("TEST");
            Serialization.AddValue(node, "v", new[] { "a", "b", "c" }, typeof(string[]));
            Assert.Equal("a,b,c", node.GetValue("v"));
            Assert.True(Serialization.TryGetValue(node, "v", out object back, typeof(string[])));
            Assert.Equal(new[] { "a", "b", "c" }, (string[])back);
        }

        [SkippableFact]
        public void AStringArrayParsesFromACommaList()
        {
            Assert.True(Serialization.TryGetValue("a, b,c", out object back, typeof(string[])));
            Assert.Equal(new[] { "a", "b", "c" }, (string[])back);
        }

        // Regression: the Rect serializer's Save wrote Rect.ToString() ("(x:1.00, y:2.00, width:30.00, height:40.00)"),
        // which its own parser (ParseExtensions.TryParseRect, "x,y,w,h") refuses.
        [SkippableFact]
        public void ARectRoundTripsThroughAConfigNode()
        {
            var node = new ConfigNode("TEST");
            var rect = new Rect(1f, 2f, 30f, 40f);
            Serialization.AddValue(node, "v", rect, typeof(Rect));
            Assert.True(Serialization.TryGetValue(node, "v", out object back, typeof(Rect)), node.GetValue("v"));
            Assert.Equal(rect, back);
        }

        [SkippableFact]
        public void ARectParsesFromFourNumbers()
        {
            Assert.True(Serialization.TryGetValue("1,2,30,40", out object back, typeof(Rect)));
            Assert.Equal(new Rect(1f, 2f, 30f, 40f), back);
        }

        [SkippableTheory]
        [InlineData(typeof(int), "often")]
        [InlineData(typeof(bool), "maybe")]
        [InlineData(typeof(Vector3), "1,2")]
        [InlineData(typeof(Color32), "1,2")]
        [InlineData(typeof(Guid), "not-a-guid")]
        [InlineData(typeof(Fruit), "Cherry")]
        public void AValueThatDoesNotParseIsRefused(Type type, string str)
        {
            Assert.False(Serialization.TryGetValue(str, out object value, type));
            Assert.Null(value);
        }

        [SkippableFact]
        public void ABadGuidOrEnumNameIsLogged()
        {
            TestLog.Instance.Clear();
            Assert.False(Serialization.TryParseGuid("not-a-guid", out Guid g));
            Assert.Equal(Guid.Empty, g);
            Assert.False(Serialization.TryGetValue("Cherry", out _, typeof(Fruit)));
            Assert.Contains(TestLog.Instance.Lines, l => l.StartsWith("Exception") && l.Contains("While trying to parse Guid"));
            Assert.Contains(TestLog.Instance.Lines, l => l.StartsWith("Exception") && l.Contains("While parsing enum"));
        }

        [SkippableFact]
        public void EnumNamesParseIgnoringCase()
        {
            Assert.True(Serialization.TryGetValue("apple", out object v, typeof(Fruit)));
            Assert.Equal(Fruit.Apple, v);
        }

        [SkippableFact]
        public void ATypeWithoutASerializerIsWrittenAsTextAndNotParsed()
        {
            var node = new ConfigNode("TEST");
            var date = new DateTime(2020, 1, 2);
            Serialization.AddValue(node, "v", date, typeof(DateTime));
            Assert.Equal(date.ToString(CultureInfo.InvariantCulture), node.GetValue("v"));
            Assert.False(Serialization.TryGetValue(node, "v", out object back, typeof(DateTime)));
            Assert.Null(back);
        }

        [SkippableFact]
        public void AMissingValueIsNotParsed()
        {
            var node = new ConfigNode("TEST");
            Assert.False(Serialization.TryGetValue(node, "nothing", out object v, typeof(int)));
            Assert.Null(v);
        }

        [SkippableFact]
        public void PatchValuesAreEditsOrIndexedEdits()
        {
            var node = new ConfigNode("TEST");
            Serialization.AddValue(node, "a", 1, typeof(int), isPatch: true);
            Serialization.AddValue(node, "b", 2, typeof(int), isPatch: true, index: 3);
            Serialization.AddValue(node, "c", Fruit.Apple, typeof(Fruit), isPatch: true);
            Assert.Equal("1", node.GetValue("%a"));
            Assert.Equal("2", node.GetValue("@b,3"));
            Assert.Equal("Apple", node.GetValue("%c"));
        }

        [SkippableFact]
        public void TopNodesAreNamedForConfigsAndPatches()
        {
            var config = new ConfigNode();
            Serialization.MakeTopNode(config, "FARConfig", "main");
            Assert.Equal("FARConfig", config.name);
            Assert.Equal("main", config.GetValue("name"));

            var plain = new ConfigNode();
            Serialization.MakeTopNode(plain, "FARConfig");
            Assert.Equal("FARConfig", plain.name);
            Assert.False(plain.HasValue("name"));

            var patch = new ConfigNode();
            Serialization.MakeTopNode(patch, "FARConfig", "main", isPatch: true);
            Assert.Equal("@FARConfig[main]:FOR[FerramAerospaceResearch]", patch.name);
            Assert.False(patch.HasValue("name"));

            var unnamed = new ConfigNode();
            Serialization.MakeTopNode(unnamed, "FARConfig", isPatch: true);
            Assert.Equal("@FARConfig:FOR[FerramAerospaceResearch]", unnamed.name);
        }

        [SkippableFact]
        public void ChildNodesCarryTheirNameAndIndex()
        {
            var parent = new ConfigNode("TEST");

            Serialization.AddNode(parent, new ConfigNode(), "ITEM", "first", index: 2);
            ConfigNode item = parent.GetNode("ITEM");
            Assert.Equal("first", item.GetValue("name"));
            Assert.Equal("2", item.GetValue("index"));

            // a name or index the child already has is kept
            var own = new ConfigNode();
            own.AddValue("name", "own");
            own.AddValue("index", 7);
            Serialization.AddNode(parent, own, "OWN", "other", index: 1);
            Assert.Equal(new[] { "own" }, parent.GetNode("OWN").GetValues("name"));
            Assert.Equal(new[] { "7" }, parent.GetNode("OWN").GetValues("index"));

            Serialization.AddNode(parent, new ConfigNode(), "ITEM", "first", isPatch: true, index: 2);
            Serialization.AddNode(parent, new ConfigNode(), "ITEM", isPatch: true, index: 0);
            Serialization.AddNode(parent, new ConfigNode(), "ITEM", "only", isPatch: true);
            Assert.True(parent.HasNode("@ITEM[first],2"), parent.ToString());
            Assert.True(parent.HasNode("@ITEM,0"), parent.ToString());
            Assert.True(parent.HasNode("@ITEM[only]"), parent.ToString());
            Assert.False(parent.GetNode("@ITEM[first],2").HasValue("name"));
        }
    }
}
