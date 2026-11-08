using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Runtime.CompilerServices;
using FerramAerospaceResearch.Interfaces;
using UnityEngine;
using Object = UnityEngine.Object;

namespace FerramAerospaceResearch.Reflection
{
    public static class ReflectionUtils
    {
        /// <summary>
        /// Cache for the loaded assemblies
        /// </summary>
        private static Assembly[] loadedAssemblies;

        /// <summary>
        /// Cache for singletons
        /// </summary>
        private static readonly ConcurrentDictionary<Type, object> instances = new ConcurrentDictionary<Type, object>();

        public static Assembly ExecutingAssembly { get; } = Assembly.GetExecutingAssembly();

        /// <summary>
        /// Cache for loaded types per assembly
        /// </summary>
        public static Dictionary<Assembly, Type[]> LoadedTypes { get; } = new Dictionary<Assembly, Type[]>();

        /// <summary>
        /// Lazy getter for loaded assemblies
        /// </summary>
        public static Assembly[] LoadedAssemblies
        {
            get { return loadedAssemblies ??= ReloadAssemblies(); }
        }

        /// <summary>
        /// Reload the assembly cache in case anything has changed
        /// </summary>
        /// <returns>Array of loaded assemblies</returns>
        public static Assembly[] ReloadAssemblies()
        {
            loadedAssemblies = AppDomain.CurrentDomain.GetAssemblies();
            return loadedAssemblies;
        }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // /// <summary>
        // /// Find types in the specified assembly that derive from T
        // /// </summary>
        // /// <param name="assembly"></param>
        // /// <param name="inherit"></param>
        // /// <typeparam name="T"></typeparam>
        // /// <returns>enumerable of Type</returns>
        // public static IEnumerable<Type> FindTypesInAssembly<T>(Assembly assembly, bool inherit = true)
        // {
        //     // ReSharper disable once LoopCanBeConvertedToQuery
        //     foreach (Type type in GetTypes(assembly))
        //         if (type.IsDefined(typeof(T), inherit))
        //             yield return type;
        // }

        // public static IEnumerable<Type> FindTypesInAssembly<T>(bool inherit = true)
        // {
        //     return FindTypesInAssembly<T>(ExecutingAssembly, inherit);
        // }

        public static IEnumerable<Type> FindTypes<T>(IEnumerable<Type> types, bool inherit)
        {
            if (inherit)
            {
                foreach (Type type in types)
                    if (typeof(T).IsBaseOf(type))
                        yield return type;
            }
            else
            {
                foreach (Type type in types)
                    if (type == typeof(T))
                        yield return type;
            }
        }

        public static IEnumerable<Type> FindTypes<T>(IEnumerable<Assembly> assemblies, bool inherit)
        {
            // ReSharper disable once LoopCanBeConvertedToQuery
            foreach (Assembly assembly in assemblies)
                foreach (Type type in FindTypes<T>(GetTypes(assembly), inherit))
                    yield return type;
        }

        public static IEnumerable<Type> FindTypes<T>(bool inherit)
        {
            return FindTypes<T>(LoadedAssemblies, inherit);
        }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public static IEnumerable<Pair<T, Type>> FindAttributeInAssembly<T>(Assembly assembly, bool inherit = true)
        //     where T : Attribute
        // {
        //     foreach (Type type in GetTypes(assembly))
        //         if (type.GetCustomAttribute(typeof(T), inherit) is T attribute)
        //             yield return Pair.Create(attribute, type);
        // }

        // public static IEnumerable<Pair<T, Type>> FindAttributeInAssembly<T>(bool inherit = true) where T : Attribute
        // {
        //     return FindAttributeInAssembly<T>(ExecutingAssembly, inherit);
        // }

        public static IEnumerable<Pair<T, Type>> FindAttribute<T>(IEnumerable<Assembly> assemblies, bool inherit = true)
            where T : Attribute
        {
            // AssemblyLoader.loadedTypes don't have all the types from all the assemblies
            foreach (Assembly assembly in assemblies)
                foreach (Type type in GetTypes(assembly))
                    if (type.GetCustomAttribute(typeof(T), inherit) is T attribute)
                        yield return Pair.Create(attribute, type);
        }

        public static IEnumerable<Pair<T, Type>> FindAttribute<T>(IEnumerable<Type> types, bool inherit = true)
            where T : Attribute
        {
            foreach (Type type in types)
                if (type.GetCustomAttribute(typeof(T), inherit) is T attribute)
                    yield return Pair.Create(attribute, type);
        }

        public static IEnumerable<Pair<T, Type>> FindAttribute<T>(bool inherit = true) where T : Attribute
        {
            return FindAttribute<T>(LoadedAssemblies, inherit);
        }

        /// <summary>
        /// Add a component to a GameObject. Note that this function must be run from the main thread since it accesses Unity methods
        /// </summary>
        /// <param name="type">type of the component</param>
        /// <param name="parent">optional parent gameobject, if not provided will create new one</param>
        /// <param name="persistant">whether this component should not be destroyed on load</param>
        /// <returns>the created component</returns>
        public static Component Create(Type type, Transform parent = null, bool persistant = false)
        {
            FARLogger.Assert(typeof(Component).IsBaseOf(type),
                             $"Invalid type given: {type.ToString()}, expected Component");

            GameObject go = parent == null ? new GameObject() : parent.gameObject;
            Component component = go.AddComponent(type);

            if (persistant)
                Object.DontDestroyOnLoad(component);

            return component;
        }

        /// <summary>
        /// Create an instance of an object from type
        /// </summary>
        /// <param name="type">type of the object</param>
        /// <param name="parent">optional parent gameobject, only used if type is a child of Component</param>
        /// <param name="persistant">whether the component should not be destroyed on load</param>
        /// <returns>the created object instance</returns>
        public static object CreateInstance(Type type, Transform parent = null, bool persistant = false)
        {
            return typeof(Component).IsBaseOf(type) ? Create(type, parent, persistant) : Activator.CreateInstance(type);
        }

        public static bool IsBaseOf(this Type type, Type derived)
        {
            return type.IsAssignableFrom(derived);
        }

        public static IEnumerable<Type> GetTypes(IEnumerable<Assembly> assemblies)
        {
            // ReSharper disable once LoopCanBeConvertedToQuery
            foreach (Assembly assembly in assemblies)
                foreach (Type type in GetTypes(assembly))
                    yield return type;
        }

        public static IEnumerable<Type> GetTypes()
        {
            return GetTypes(LoadedAssemblies);
        }

        public static Type[] GetTypes(Assembly assembly)
        {
            if (LoadedTypes.TryGetValue(assembly, out Type[] types))
                return types;

            try
            {
                types = assembly.GetTypes();
            }
            catch (ReflectionTypeLoadException e)
            {
                FARLogger.Exception(e, $"While loading {assembly}");
                foreach (Exception exception in e.LoaderExceptions)
                {
                    FARLogger.Exception(exception);
                }

                types = e.Types.Where(t => t != null).ToArray();
            }
            catch (Exception e)
            {
                FARLogger.Exception(e);
                types = Type.EmptyTypes;
            }

            LoadedTypes.Add(assembly, types);

            return types;
        }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // public static Type[] ReloadTypes(Assembly assembly)
        // {
        //     LoadedTypes.Remove(assembly);
        //     return GetTypes(assembly);
        // }

        public static IEnumerable<Pair<T, FieldInfo>> GetFieldsWithAttribute<T>(
            this Type type,
            bool compilerGenerated = false,
            BindingFlags flags = BindingFlags.Instance | BindingFlags.NonPublic
        ) where T : Attribute
        {
            // ReSharper disable once LoopCanBeConvertedToQuery
            foreach (FieldInfo fi in type.GetFields(flags))
            {
                if (!compilerGenerated && fi.GetCustomAttribute<CompilerGeneratedAttribute>() != null)
                    continue;

                T attribute = fi.GetCustomAttribute<T>();
                if (attribute != null)
                    yield return Pair.Create(attribute, fi);
            }
        }

        public static IEnumerable<Pair<T, PropertyInfo>> GetPropertiesWithAttribute<T>(
            this Type type,
            BindingFlags flags = BindingFlags.Instance | BindingFlags.NonPublic
        ) where T : Attribute
        {
            // ReSharper disable once LoopCanBeConvertedToQuery
            foreach (PropertyInfo fi in type.GetProperties(flags))
            {
                T attribute = fi.GetCustomAttribute<T>();
                if (attribute != null)
                    yield return Pair.Create(attribute, fi);
            }
        }

        public static IEnumerable<Type> GetParentTypes(this Type type)
        {
            // is there any base type?
            if (type == null)
                yield break;

            // return all implemented or inherited interfaces
            foreach (Type i in type.GetInterfaces())
                yield return i;

            // return all inherited types
            Type currentBaseType = type.BaseType;
            while (currentBaseType != null)
            {
                yield return currentBaseType;
                currentBaseType = currentBaseType.BaseType;
            }
        }

        /// <summary>
        /// Cache a singleton instance
        /// </summary>
        /// <param name="type">type of the instance</param>
        /// <param name="instance"></param>
        /// <returns>instance</returns>
        public static object RegisterInstance(Type type, object instance)
        {
            if (!instances.TryAdd(type, instance))
                FARLogger.ErrorFormat("Could not register instance <{0}> of type {1}", instance, type.Name);
            else if (instance != null && type != instance.GetType() && !instances.ContainsKey(instance.GetType()))
                if (!instances.TryAdd(instance.GetType(), instance))
                    FARLogger.ErrorFormat("Could not register instance <{0}> of type {1}",
                                          instance,
                                          instance.GetType().Name);
            return instance;
        }

        /// <summary>
        /// Try to get a singleton instance. The resolution order is:
        ///     1. Type is a static class
        ///     2. An instance has been already cached with the provided type
        ///     3. An instance has been already cached with a different derived type
        ///     4. Type contains static field 'instance' (case insensitive)
        ///     5. Type contains static property 'instance' (case insensitive)
        ///     6. Type contains static method 'instance' (case insensitive)
        /// New found instances are cached
        /// </summary>
        /// <param name="type">type of the singleton</param>
        /// <returns>instance if found and not static class, null otherwise</returns>
        public static object FindInstance(Type type)
        {
            const BindingFlags flags = BindingFlags.Static |
                                       BindingFlags.NonPublic |
                                       BindingFlags.Public |
                                       BindingFlags.FlattenHierarchy |
                                       BindingFlags.IgnoreCase;

            // short circuit for static types
            if (type.IsStatic())
                return null;

            // first try checking if an instance has already been found
            if (instances.TryGetValue(type, out object instance))
                return instance;

            // otherwise do a slow check with inheritance
            foreach (KeyValuePair<Type, object> pair in instances)
            {
                if (type.IsBaseOf(pair.Key))
                    return pair.Value;
            }

            // not found so check for fields/properties/methods 'Instance'
            FieldInfo field = type.GetField("Instance", flags);
            if (field != null)
            {
                instance = field.GetValue(null);
                if (type.IsBaseOf(instance?.GetType()))
                    return RegisterInstance(type, instance);
                FARLogger.TraceFormat("Found static field Instance in {0} but it returns a different type {1}",
                                      type,
                                      instance?.GetType());
            }

            PropertyInfo info = type.GetProperty("Instance", flags);
            if (info != null)
            {
                instance = info.GetValue(null, flags, null, null, null);
                if (type.IsBaseOf(instance?.GetType()))
                    return RegisterInstance(type, instance);
                FARLogger.TraceFormat("Found static property Instance in {0} but it returns a different type {1}",
                                      type,
                                      instance?.GetType());
            }

            MethodInfo method = type.GetMethod("Instance", flags, null, new Type[0], null);
            if (method == null)
                return RegisterInstance(type, null);

            // ReSharper disable once AssignNullToNotNullAttribute - false positive
            instance = method.Invoke(null, flags, null, new object[0], null);
            if (instance != null && type.IsBaseOf(instance.GetType()))
                return RegisterInstance(type, instance);

            FARLogger.TraceFormat("Found static method Instance in {0} but it returns a different type {1}",
                                  type,
                                  instance?.GetType());
            return RegisterInstance(type, null);
        }

        public static bool IsStatic(this Type type)
        {
            return type.IsAbstract && type.IsSealed;
        }

        /// <summary>
        /// Get generic interface type parameters
        /// </summary>
        /// <param name="type">type implementing the interface</param>
        /// <param name="interfaceType">generic interface type</param>
        /// <returns>array of generic arguments of interfaceType if it is found in type, otherwise null</returns>
        public static Type[] GetGenericInterfaceArguments(Type type, Type interfaceType)
        {
            if (type.IsGenericType && type.GetGenericTypeDefinition() == interfaceType)
                return type.GetGenericArguments();

            // ReSharper disable once LoopCanBeConvertedToQuery
            foreach (Type i in type.GetInterfaces())
                if (i.IsGenericType && i.GetGenericTypeDefinition() == interfaceType)
                    return i.GetGenericArguments();

            return null;
        }

        /// <summary>
        /// See <see cref="GetGenericInterfaceArguments"/> for description of parameters
        /// </summary>
        /// <returns>Returns the first generic interface argument</returns>
        public static Type InterfaceParameter(Type type, Type interfaceType)
        {
            Type[] args = GetGenericInterfaceArguments(type, interfaceType);

            return args?.Single();
        }

        /// <summary>
        /// Get the IConfigValue generic type parameter
        /// </summary>
        /// <param name="type">type to check for IConfigValue interface</param>
        /// <returns>Generic parameter if found, null otherwise</returns>
        public static Type ConfigValueType(Type type)
        {
            return InterfaceParameter(type, typeof(IConfigValue<>));
        }

        /// <summary>
        /// Get the IList generic type parameter
        /// </summary>
        /// <param name="type">type to check for List interface</param>
        /// <returns>Generic parameter if found, null otherwise</returns>
        public static Type ListType(Type type)
        {
            return InterfaceParameter(type, typeof(IList<>));
        }

        // Unused: nothing in FAR calls it (commented out after the test suite's coverage run)
        // /// <summary>
        // /// Check if type implements generic IList interface
        // /// </summary>
        // /// <param name="type">type to check</param>
        // /// <returns></returns>
        // public static bool IsListValue(Type type)
        // {
        //     return ListType(type) != null;
        // }
    }
}
