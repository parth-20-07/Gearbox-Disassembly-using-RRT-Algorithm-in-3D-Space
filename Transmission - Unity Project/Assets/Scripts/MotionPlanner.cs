using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using Random = System.Random;

public class MotionPlanner : MonoBehaviour
{
    #region Variables

    [Header("GameObjects")]
    // All the Game-objects needed to interact with environment
    [SerializeField]
    private GameObject mainShaft;

    // [SerializeField] private GameObject groundPlane;
    [SerializeField] private GameObject goalMainShaft;
    private GameObject _collisionDetectionObject;

    // Physics Objects
    private List<Collider> _collisionDetectionShaftColliderList;
    private List<Collider> _obstacleColliderList;

    [Header("RRT Parameters")]
    //Parameters needed to plan the RRT
    [SerializeField]
    [Range(1, 100)]
    private int nodeSearchRadiusInMm;

    private class ValueRange
    {
        public int Min { get; }
        public int Max { get; }

        public ValueRange(int minimum, int maximum)
        {
            Min = minimum;
            Max = maximum;
        }
    }

    private readonly ValueRange _xPositionRange = new(-881, 276);
    private readonly ValueRange _yPositionRange = new(100, 820);
    private readonly ValueRange _zPositionRange = new(0, 0);
    private readonly ValueRange _xRotationRange = new(0, 0);
    private readonly ValueRange _yRotationRange = new(0, 0);
    private readonly ValueRange _zRotationRange = new(-90, 90);

    // Update Function Flags
    private bool _endPositionHeightReached, _endRotationReached, _rrtPathTraced, _rrtPathTracingComplete,dataNotSaved;

    private class RrtNode
    {
        private readonly GameObject _node;
        private readonly RrtNode _parentNode;

        public RrtNode(Vector3 position, Quaternion rotation, RrtNode parent, Transform parentTransform,
            int mappingLayer, Material mappingMaterial)
        {
            _node = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            _node.transform.position = position;
            _node.transform.rotation = rotation;
            _node.transform.parent = parentTransform;
            _node.transform.localScale = new Vector3(5.0f, 5.0f, 5.0f);
            _node.layer = mappingLayer;
            _node.GetComponent<MeshRenderer>().material = mappingMaterial;

            _parentNode = parent;
            if (_parentNode != null)
            {
                LineRenderer edge = _node.AddComponent<LineRenderer>();
                edge.SetPosition(0, parent.GetTransform().position);
                edge.SetPosition(1, _node.transform.position);
                edge.gameObject.layer = mappingLayer;
                edge.gameObject.GetComponent<MeshRenderer>().material = mappingMaterial;
            }
        }

        public Transform GetTransform()
        {
            return _node.transform;
        }

        public RrtNode SelectParent()
        {
            _node.GetComponent<MeshRenderer>().material.color = Color.white;
            _node.layer = 0;
            var edge = _node.GetComponent<LineRenderer>();
            if (edge != null)
            {
                edge.gameObject.GetComponent<MeshRenderer>().material.color = Color.white;
                edge.gameObject.layer = 0;
            }

            return this._parentNode;
        }
    }

    private List<RrtNode> _rrtGraph, _rrtPath;

    [Header("Animation Parameters")]
    // Parameters needed to better visualization and animations
    [SerializeField]
    private float animationSpeedInMmps;

    [SerializeField] private Material collisionDetectorMaterial;
    [SerializeField] private string mappingLayer;
    private LayerMask _mappingLayerNumber;

    #endregion Variables

    #region Main Methods

    private void Start()
    {
        _mappingLayerNumber = LayerMask.NameToLayer(mappingLayer);

        _collisionDetectionObject = Instantiate(mainShaft, this.transform, true);
        _collisionDetectionObject.transform.name = "Collision Detector";
        _collisionDetectionObject.layer = _mappingLayerNumber;
        var collisionDetectorTransformList = _collisionDetectionObject.GetComponentsInChildren<Transform>().ToList();
        foreach (var cdTransform in collisionDetectorTransformList)
        {
            cdTransform.gameObject.layer = _mappingLayerNumber;
            if (cdTransform.gameObject.GetComponent<MeshRenderer>() != null)
                cdTransform.gameObject.GetComponent<MeshRenderer>().material = collisionDetectorMaterial;
        }

        _collisionDetectionShaftColliderList = _collisionDetectionObject.GetComponentsInChildren<Collider>().ToList();

        var mainShaftColliderList = mainShaft.GetComponentsInChildren<Collider>().ToList();
        _obstacleColliderList = GetComponentsInChildren<Collider>().ToList();

        for (var i = 0; i < mainShaftColliderList.Count; i++)
        {
            _obstacleColliderList.Remove(mainShaftColliderList[i]);
            _obstacleColliderList.Remove(_collisionDetectionShaftColliderList[i]);
        }

        // var groundPlaneCollider = groundPlane.GetComponent<Collider>();
        // _obstacleColliderList.Add(groundPlaneCollider);

        _rrtGraph = new List<RrtNode>();
        _rrtPath = new List<RrtNode>();

        RrtNode startNode = new(mainShaft.transform.position, mainShaft.transform.rotation, null, transform,
            _mappingLayerNumber, collisionDetectorMaterial);
        _rrtGraph.Add(startNode);

        _endPositionHeightReached = false;
    }

    private void Update()
    {
        if (!_endPositionHeightReached)
        {
            /*
            // RRT Pseudo Code
            * Q_goal //region that identifies success
            * Counter = 0 //keeps track of iterations
            * lim = n //number of iterations algorithm should run for
            * G(V,E) //Graph containing edges and vertices, initialized as empty
            * While counter < lim:
            * * * X_new  = RandomPosition()
            * * * if IsInObstacle(X_new) == True:
            * * * * * * continue
            * * * X_nearest = Nearest(G(V,E), X_new) //find nearest vertex
            * * * Link = Chain(X_new, X_nearest)
            * * * G.append(Link)
            * * * if X_new in Q_goal:
            * * * * * * Return G
            * Return G
            */
            Vector3 newPosition;
            Quaternion newRotation;
            RrtNode closestNode;
            (closestNode, newPosition, newRotation) = GetRandomConfiguration();

            if (DoesConfigurationAlreadyExists(newPosition, newRotation))
                return;
            if (IsConfigurationInCollision(newPosition, newRotation))
                return;
            if (DoesNewConfigurationPathContainObstacle(newPosition, newRotation, closestNode.GetTransform()))
                return;

            var node = new RrtNode(newPosition, newRotation, closestNode, transform, _mappingLayerNumber,
                collisionDetectorMaterial);
            _rrtGraph.Add(node);
            // Debug.Log("Position: " + newPosition + "Rotation(Euler): " + newRotation.eulerAngles);
            if (Math.Abs(newPosition.y - goalMainShaft.transform.position.y) < 10.0f)
                _endPositionHeightReached = true;
        }
        else if (_endPositionHeightReached && !_endRotationReached)
        {
            var lastNode = _rrtGraph.Last();

            var goalRotation = goalMainShaft.transform.rotation;
            var goalPosition = goalMainShaft.transform.position;

            var endNode = new RrtNode(goalPosition, goalRotation, lastNode, transform, _mappingLayerNumber,
                collisionDetectorMaterial);
            _rrtGraph.Add(endNode);

            _endRotationReached = true;
        }
        else if (_endRotationReached && !_rrtPathTraced)
        {
            Destroy(_collisionDetectionObject);
            var lastNode = _rrtGraph.Last();
            while (lastNode != null)
            {
                _rrtPath.Add(lastNode);
                lastNode = lastNode.SelectParent();
            }

            _rrtPath.Reverse();
            _rrtPathTraced = true;
            StartCoroutine(TravelOnPath());
        }
        else if (_rrtPathTraced && _rrtPathTracingComplete && !dataNotSaved)
        {
            StopAllCoroutines();
            GenerateDataForPlotting("rrtPath.csv",_rrtPath);
            GenerateDataForPlotting("rrtGraph.csv",_rrtGraph);
            Debug.Log("Data Export Complete");
            dataNotSaved = true;
            Application.Quit();
        }
    }

    #endregion Main Methods

    #region Custom Methods

    private (RrtNode closestNode, Vector3 position, Quaternion rotation) GetRandomConfiguration()
    {
        var rnd = new Random();

        var xPos = rnd.Next(_xPositionRange.Min, _xPositionRange.Max);
        var yPos = rnd.Next(_yPositionRange.Min, _yPositionRange.Max);
        var zPos = rnd.Next(_zPositionRange.Min, _zPositionRange.Max);
        var position = new Vector3(xPos, yPos, zPos);

        var xRot = rnd.Next(_xRotationRange.Min, _xRotationRange.Max);
        var yRot = rnd.Next(_yRotationRange.Min, _yRotationRange.Max);
        var zRot = rnd.Next(_zRotationRange.Min, _zRotationRange.Max);
        Vector3 eulerRotation = new(xRot, yRot, zRot);

        var closestNode = FindClosestNode(position);

        var directionVal = (position - closestNode.GetTransform().position).normalized;
        var newPosition = closestNode.GetTransform().position + (nodeSearchRadiusInMm * directionVal);
        newPosition = new((int)newPosition.x, (int)newPosition.y, (int)newPosition.z);

        Vector3 newRotation = closestNode.GetTransform().rotation.eulerAngles +
                              new Vector3(eulerRotation.x / 20.0f, eulerRotation.y / 20.0f, eulerRotation.z / 20.0f);
        newRotation = new((int)newRotation.x, (int)newRotation.y, (int)newRotation.z);

        return (closestNode, newPosition, Quaternion.Euler(newRotation));
    }

    private bool DoesConfigurationAlreadyExists(Vector3 newPosition, Quaternion newRotation)
    {
        foreach (var rrtNode in _rrtGraph)
        {
            var position = rrtNode.GetTransform().position;
            var rotation = rrtNode.GetTransform().rotation;
            if ((position == newPosition) && (rotation == newRotation)) return true;
        }

        return false;
    }

    private RrtNode FindClosestNode(Vector3 randomPosition)
    {
        var distance = int.MaxValue;
        var nodeIndex = int.MaxValue;
        for (var i = 0; i < _rrtGraph.Count; i++)
        {
            var node = _rrtGraph[i];
            var nodePosition = node.GetTransform().position;
            var currentDistance = (int)(nodePosition - randomPosition).magnitude;
            if (currentDistance < distance)
            {
                distance = currentDistance;
                nodeIndex = i;
            }
        }

        return _rrtGraph[nodeIndex];
    }

    private bool IsConfigurationInCollision(Vector3 newPosition, Quaternion newRotation)
    {
        _collisionDetectionObject.transform.position = newPosition;
        _collisionDetectionObject.transform.rotation = newRotation;
        foreach (var collisionDetectionShaftCollider in _collisionDetectionShaftColliderList)
        {
            foreach (var obstacleCollider in _obstacleColliderList)
            {
                // if (collisionDetectionShaftCollider.bounds.Intersects(obstacleCollider.bounds))
                // {
                //     return true;
                // }
                if (Physics.ComputePenetration(obstacleCollider, obstacleCollider.transform.position,
                        obstacleCollider.transform.rotation,
                        collisionDetectionShaftCollider, collisionDetectionShaftCollider.transform.position,
                        collisionDetectionShaftCollider.transform.rotation,
                        out _, out _))
                {
                    return true;
                }
            }
        }

        return false;
    }

    private bool DoesNewConfigurationPathContainObstacle(Vector3 newConfigurationPosition,
        Quaternion newConfigurationRotation, Transform lastNodeTransform)
    {
        var lastConfigurationPosition = lastNodeTransform.position;
        var lastConfigurationRotation = lastNodeTransform.rotation;

        for (float i = 0; i < 1; i += 0.2f)
        {
            var interpolatedPosition = Vector3.Lerp(lastConfigurationPosition, newConfigurationPosition, i);
            var interpolatedRotation = Quaternion.Lerp(lastConfigurationRotation, newConfigurationRotation, i);
            if (IsConfigurationInCollision(interpolatedPosition, interpolatedRotation))
                return true;
        }

        return false;
    }

    private IEnumerator TravelOnPath()
    {
        int i = 0;
        var startNode = _rrtPath[0];
        var startPosition = startNode.GetTransform().position;
        var startRotation = startNode.GetTransform().rotation;

        var nextNode = _rrtPath[1];
        var nextPosition = nextNode.GetTransform().position;
        var nextRotation = nextNode.GetTransform().rotation;
        float time = 0.0f;

        float travelDistance = (startPosition - nextPosition).sqrMagnitude;
        float timeStep = travelDistance / animationSpeedInMmps;

        while (true)
        {
            Vector3 position = Vector3.Lerp(startPosition, nextPosition, time);
            Quaternion rotation = Quaternion.Lerp(startRotation, nextRotation, time);
            mainShaft.transform.position = position;
            mainShaft.transform.rotation = rotation;
            time += (1 / timeStep);
            if (time >= 1.0f)
            {
                i++;

                if (i == _rrtPath.Count)
                {
                    _rrtPathTracingComplete = true;
                    break;
                }

                startNode = nextNode;
                startPosition = startNode.GetTransform().position;
                startRotation = startNode.GetTransform().rotation;

                nextNode = _rrtPath[i];
                nextPosition = nextNode.GetTransform().position;
                nextRotation = nextNode.GetTransform().rotation;

                travelDistance = (startPosition - nextPosition).sqrMagnitude;
                timeStep = travelDistance / animationSpeedInMmps;
                time = 0.0f;
            }

            yield return null;
        }
    }

    private void GenerateDataForPlotting(string pathFile, List<RrtNode> nodeList)
    {
        TextWriter tw = new StreamWriter(pathFile, true);
        // Final Path Plot
        tw.WriteLine("x,y,z");
        foreach (var node in nodeList)
        {
            var position = node.GetTransform().position;
            tw.WriteLine(position.x.ToString() + ',' + position.y.ToString() + ',' + position.z.ToString());
        }
        tw.Close();
    }

    #endregion Custom Methods
}