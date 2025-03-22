package main

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"strconv"
	"sync"
	"time"
)

type TimelineData struct {
	TimelineEdits []TimelineEdit `json:"timelineEdits"`
}

type TimelineEdit struct {
	DeviceID        string           `json:"deviceId"`
	RawSignal       *RawSignal       `json:"rawSignal,omitempty"`
	PlaceAggregates *PlaceAggregates `json:"placeAggregates,omitempty"`
	Description     string           `json:"description,omitempty"`
	Tags            []string         `json:"tags,omitempty"`
}

type PlaceAggregates struct {
	PlaceAggregateInfo      []PlaceAggregate `json:"placeAggregateInfo"`
	ProcessWindow           ProcessWindow    `json:"processWindow"`
	WindowSizeHrs           int              `json:"windowSizeHrs"`
	TopRankedPlacesPlaceIds []string         `json:"topRankedPlacesPlaceIds"`
	Metadata                Metadata         `json:"metadata"`
}

type ProcessWindow struct {
	StartTime string `json:"startTime"`
	EndTime   string `json:"endTime"`
}

type PlaceAggregate struct {
	Point struct {
		LatE7 int `json:"latE7"`
		LngE7 int `json:"lngE7"`
	} `json:"point"`
	PlaceID    string `json:"placeId"`
	PlacePoint struct {
		LatE7 int `json:"latE7"`
		LngE7 int `json:"lngE7"`
	} `json:"placePoint"`
}

type RawSignal struct {
	Signal              Signal   `json:"signal"`
	AdditionalTimestamp string   `json:"additionalTimestamp"`
	Metadata            Metadata `json:"metadata"`
}

type Signal struct {
	ActivityRecord *ActivityRecord `json:"activityRecord,omitempty"`
	Position       *Position       `json:"position,omitempty"`
	WifiScan       *WifiScan       `json:"wifiScan,omitempty"`
}

type Position struct {
	Point                Point   `json:"point"`
	AccuracyMm           int     `json:"accuracyMm"`
	AltitudeMeters       float64 `json:"altitudeMeters,omitempty"`
	Source               string  `json:"source"`
	Timestamp            string  `json:"timestamp"`
	SpeedMetersPerSecond float64 `json:"speedMetersPerSecond,omitempty"`
}

type WifiScan struct {
	DeliveryTime string `json:"deliveryTime"`
}

type ActivityRecord struct {
	DetectedActivities []DetectedActivity `json:"detectedActivities"`
	Timestamp          string             `json:"timestamp"`
}

type DetectedActivity struct {
	ActivityType string  `json:"activityType"`
	Probability  float64 `json:"probability"`
}

type Point struct {
	LatE7 int `json:"latE7"`
	LngE7 int `json:"lngE7"`
}

type Metadata struct {
	Platform string `json:"platform"`
}

type PositionData struct {
	ID          string     `json:"id"`
	Lat         float64    `json:"lat"`
	Lon         float64    `json:"lon"`
	AccuracyMm  int        `json:"accuracyMm"`
	Altitude    float64    `json:"altitude,omitempty"`
	Source      string     `json:"source"`
	Speed       float64    `json:"speed,omitempty"`
	Timestamp   time.Time  `json:"timestamp"`
	DeviceID    string     `json:"deviceId"`
	Description string     `json:"description,omitempty"`
	Tags        []string   `json:"tags,omitempty"`
	IsUserAdded bool       `json:"isUserAdded"`
	RawSignal   *RawSignal `json:"rawSignal,omitempty"`
}

var (
	positions   []PositionData
	positionsMu sync.RWMutex
	dataFiles   = []string{"TimelineEdits.json", "TimelineNewEdits.json"}
)

func main() {
	// Load data from both files
	for _, file := range dataFiles {
		loadData(file)
	}

	mux := http.NewServeMux()
	mux.HandleFunc("/", handleCORS(handleIndex))
	mux.HandleFunc("/api/positions", handleCORS(handlePositions))
	mux.HandleFunc("/api/routes", handleCORS(handleRoutes))
	mux.HandleFunc("/api/add-point", handleCORS(handleAddPoint))
	mux.HandleFunc("/api/remove-point", handleCORS(handleRemovePoint))

	port := "8080"
	log.Printf("Server running on :%s", port)
	log.Fatal(http.ListenAndServe(":"+port, mux))
}

func handleCORS(handler http.HandlerFunc) http.HandlerFunc {
	return func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Access-Control-Allow-Origin", "*")
		w.Header().Set("Access-Control-Allow-Methods", "POST, GET, OPTIONS, DELETE")
		w.Header().Set("Access-Control-Allow-Headers", "Content-Type")

		if r.Method == "OPTIONS" {
			w.WriteHeader(http.StatusOK)
			return
		}

		handler(w, r)
	}
}

func loadData(filename string) {
	data, err := os.ReadFile(filename)
	if os.IsNotExist(err) {
		log.Printf("File %s does not exist, skipping", filename)
		return
	}
	if err != nil {
		log.Printf("Error reading %s: %v", filename, err)
		return
	}

	var root TimelineData
	if err := json.Unmarshal(data, &root); err != nil {
		log.Printf("Error parsing %s: %v", filename, err)
		return
	}

	positionsMu.Lock()
	defer positionsMu.Unlock()

	currentID := len(positions)

	for _, edit := range root.TimelineEdits {
		// Process rawSignal positions
		if edit.RawSignal != nil && edit.RawSignal.Signal.Position != nil {
			pos := edit.RawSignal.Signal.Position
			ts, err := time.Parse(time.RFC3339, pos.Timestamp)
			if err != nil {
				log.Printf("Invalid timestamp format: %s", pos.Timestamp)
				ts = time.Now()
			}

			positions = append(positions, PositionData{
				ID:          strconv.Itoa(currentID),
				Lat:         float64(pos.Point.LatE7) / 1e7,
				Lon:         float64(pos.Point.LngE7) / 1e7,
				AccuracyMm:  pos.AccuracyMm,
				Altitude:    pos.AltitudeMeters,
				Source:      pos.Source,
				Speed:       pos.SpeedMetersPerSecond,
				Timestamp:   ts,
				DeviceID:    edit.DeviceID,
				Description: edit.Description,
				Tags:        edit.Tags,
				IsUserAdded: filename == "TimelineNewEdits.json",
				RawSignal:   edit.RawSignal,
			})
			currentID++
		}

		// Process placeAggregates positions
		if edit.PlaceAggregates != nil {
			var ts time.Time
			var err error

			// Use process window start time if available
			if edit.PlaceAggregates.ProcessWindow.StartTime != "" {
				ts, err = time.Parse(time.RFC3339, edit.PlaceAggregates.ProcessWindow.StartTime)
				if err != nil {
					log.Printf("Invalid processWindow startTime: %s", edit.PlaceAggregates.ProcessWindow.StartTime)
					ts = time.Now()
				}
			} else {
				ts = time.Now()
			}

			for _, place := range edit.PlaceAggregates.PlaceAggregateInfo {
				positions = append(positions, PositionData{
					ID:          strconv.Itoa(currentID),
					Lat:         float64(place.Point.LatE7) / 1e7,
					Lon:         float64(place.Point.LngE7) / 1e7,
					DeviceID:    edit.DeviceID,
					Timestamp:   ts,
					Description: edit.Description,
					Tags:        edit.Tags,
					IsUserAdded: filename == "TimelineNewEdits.json",
				})
				currentID++
			}
		}
	}

	log.Printf("Loaded %d positions from %s", len(root.TimelineEdits), filename)
}

// [Keep all the handler functions from your working version]
// [Include handleIndex, handlePositions, handleRoutes, handleAddPoint, handleRemovePoint]
// [Include saveUserEdits function and other helper functions]

// Rest of the handlers remain identical to your working version with user-added points functionality

func handleIndex(w http.ResponseWriter, r *http.Request) {
	// Only serve index.html for root path
	if r.URL.Path != "/" {
		http.NotFound(w, r)
		return
	}

	fmt.Fprint(w, `<!DOCTYPE html>
<html>
<head>
	<title>Timeline Map</title>
	<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
	<style>
		body { margin: 0; padding: 0; }
		#map { height: 100vh; width: 100%; }
		.loading { 
			position: fixed; 
			top: 10px; 
			left: 50%; 
			transform: translateX(-50%); 
			padding: 10px; 
			background: white; 
			box-shadow: 0 0 5px rgba(0,0,0,0.3);
			z-index: 1000;
			border-radius: 4px;
		}
		.error {
			position: fixed;
			top: 10px;
			left: 50%;
			transform: translateX(-50%);
			padding: 10px;
			background: #ffeeee;
			color: #cc0000;
			box-shadow: 0 0 5px rgba(0,0,0,0.3);
			z-index: 1000;
			border-radius: 4px;
		}
		.toolbar {
			position: fixed;
			bottom: 20px;
			left: 50%;
			transform: translateX(-50%);
			background: white;
			padding: 10px;
			border-radius: 4px;
			box-shadow: 0 0 5px rgba(0,0,0,0.3);
			z-index: 1000;
			display: flex;
			gap: 10px;
		}
		.toolbar button {
			padding: 5px 10px;
			cursor: pointer;
		}
	</style>
</head>
<body>
	<div id="map"></div>
	<div class="loading" id="loading">Loading map data...</div>
	<div class="error" id="error" style="display:none;"></div>
	<div class="toolbar">
		<button id="refreshBtn">Refresh Data</button>
		<button id="addPointBtn">Add Point (or click map)</button>
	</div>
	
	<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
	<script>
		// Global variables
		let map = null;
		let markers = [];
		let routes = [];
		let updateTimeout;
		let isDataLoaded = false;
		
		// Base map layers
		const baseLayers = {
			"OpenStreetMap": L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
				attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
			}),
			"Satellite": L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
				attribution: 'Esri, Maxar, Earthstar Geographics, and the GIS User Community'
			}),
			"Grayscale": L.tileLayer('https://tiles.wmflabs.org/bw-mapnik/{z}/{x}/{y}.png', {
				attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
			})
		};

		// Initialize map when DOM is ready
		document.addEventListener('DOMContentLoaded', () => {
			initMap();
			
			// Setup event listeners for toolbar buttons
			document.getElementById('refreshBtn').addEventListener('click', () => {
				updateMap(true);
			});
			
			document.getElementById('addPointBtn').addEventListener('click', () => {
				showMessage('Click on the map to add a point');
			});
		});

		// Initialize Leaflet map
		function initMap() {
			try {
				// Create map with default center
				map = L.map('map', {
					center: [50.0, 10.0],
					zoom: 5,
					layers: [baseLayers.OpenStreetMap]
				});
				
				// Add layer control
				L.control.layers(baseLayers).addTo(map);
				
				// Add scale control
				L.control.scale().addTo(map);
				
				// Register map event listeners
				map.on('moveend', () => {
					clearTimeout(updateTimeout);
					updateTimeout = setTimeout(() => updateMap(false), 300);
				});

				map.on('click', (e) => addPointAtLocation(e.latlng));
				
				// Load initial data
				updateMap(true).then(() => {
					// If we have points, zoom to fit them
					if (markers.length > 0) {
						const group = L.featureGroup(markers);
						map.fitBounds(group.getBounds(), { padding: [50, 50] });
					}
				});
				
			} catch (err) {
				showError('Map initialization failed: ' + err.message);
				console.error('Map initialization failed:', err);
			}
		}

		// Add a new point at a specific location
		async function addPointAtLocation(latlng) {
			const desc = prompt('Enter description:');
			if (!desc) return; // User cancelled
			
			const tags = prompt('Enter tags (comma separated):') || '';
			
			showLoading('Adding point...');
			try {
				const response = await fetch('/api/add-point', {
					method: 'POST',
					headers: { 'Content-Type': 'application/json' },
					body: JSON.stringify({
						lat: latlng.lat,
						lon: latlng.lng,
						description: desc,
						tags: tags.split(',').map(t => t.trim()).filter(t => t)
					})
				});
				
				if (!response.ok) {
					throw new Error('Server returned ' + response.status);
				}
				
				await updateMap(true);
				showMessage('Point added successfully', 2000);
			} catch (err) {
				showError('Failed to add point: ' + err.message);
				console.error('Add point failed:', err);
			} finally {
				hideLoading();
			}
		}
		
		// Delete a point by ID
		async function deletePoint(id) {
			if (!confirm('Are you sure you want to delete this point?')) {
				return;
			}
			
			showLoading('Deleting point...');
			try {
				const response = await fetch('/api/remove-point?id=' + id, { 
					method: 'POST' 
				});
				
				if (!response.ok) {
					throw new Error('Server returned ' + response.status);
				}
				
				await updateMap(true);
				showMessage('Point deleted successfully', 2000);
			} catch (err) {
				showError('Failed to delete point: ' + err.message);
				console.error('Delete failed:', err);
			} finally {
				hideLoading();
			}
		}

		// Update map with latest data
		async function updateMap(forceRefresh = false) {
			showLoading();
			try {
				// Only clear and reload everything if this is a forced refresh
				if (forceRefresh) {
					// Clear existing markers
					markers.forEach(marker => map.removeLayer(marker));
					markers = [];
					
					// Clear existing routes
					routes.forEach(route => map.removeLayer(route));
					routes = [];
				}
				
				// Load positions
				const positionsRes = await fetchWithTimeout('/api/positions', 10000);
				if (!positionsRes.ok) {
					throw new Error('Server returned ' + positionsRes.status);
				}
				
				const positionsData = await positionsRes.json();
				
				// Add markers for positions if they don't exist
				positionsData.features.forEach(f => {
					const lat = f.geometry.coordinates[1];
					const lng = f.geometry.coordinates[0];
					const id = f.properties.id;
					
					// Skip if we already have this marker and not forced refresh
					if (!forceRefresh && markers.some(m => m.id === id)) {
						return;
					}
					
					const marker = L.marker([lat, lng])
						.bindPopup(createPopupContent(f.properties));
					
					marker.id = id; // Store ID for reference
					marker.addTo(map);
					markers.push(marker);
				});
				
				// Load routes if forced refresh
				if (forceRefresh) {
					const routesRes = await fetchWithTimeout('/api/routes', 10000);
					if (!routesRes.ok) {
						throw new Error('Server returned ' + routesRes.status);
					}
					
					const routesData = await routesRes.json();
					
					Object.entries(routesData).forEach(([day, coords]) => {
						// Convert longitude/latitude format to latitude/longitude for Leaflet
						const points = coords.map(c => [c[1], c[0]]);
						
						const polyline = L.polyline(points, {
							color: getRandomColor(),
							weight: 3,
							opacity: 0.7
						}).bindPopup('Day: ' + day);
						
						polyline.addTo(map);
						routes.push(polyline);
					});
				}
				
				isDataLoaded = true;
				hideError();
				
			} catch (err) {
				showError('Failed to update map: ' + err.message);
				console.error('Map update failed:', err);
			} finally {
				hideLoading();
			}
		}
		
		// Generate a random color for routes
		function getRandomColor() {
			const colors = ['#3388ff', '#ff3333', '#33ff33', '#ff33ff', '#ffff33', '#33ffff', '#ff8833', '#8833ff'];
			return colors[Math.floor(Math.random() * colors.length)];
		}

		// Create HTML content for marker popups
		function createPopupContent(props) {
			let content = '<div style="min-width: 200px">';
			
			// Format the timestamp nicely if available
			let timestamp = props.timestamp || 'No timestamp';
			if (timestamp !== 'No timestamp') {
				try {
					const date = new Date(timestamp);
					timestamp = date.toLocaleString();
				} catch (e) {
					// Keep original if parsing fails
				}
			}
			
			content += '<h3>' + timestamp + '</h3>';
			content += '<p><strong>ID:</strong> ' + props.id + '</p>';
			content += '<p><strong>Location:</strong> ' + props.lat.toFixed(6) + ', ' + props.lng.toFixed(6) + '</p>';
			
			if (props.deviceId) {
				content += '<p><strong>Device:</strong> ' + props.deviceId + '</p>';
			}
			
			if (props.accuracy) {
				content += '<p><strong>Accuracy:</strong> ' + props.accuracy + 'mm</p>';
			}
			
			if (props.source) {
				content += '<p><strong>Source:</strong> ' + props.source + '</p>';
			}
			
			if(props.description) {
				content += '<p><strong>Description:</strong> ' + props.description + '</p>';
			}
			
			if(props.tags && props.tags.length > 0) {
				content += '<p><strong>Tags:</strong> ' + props.tags.join(', ') + '</p>';
			}
			
			if(props.isUserAdded) {
				content += '<p><button onclick="deletePoint(\'' + props.id + '\')" style="background-color: #ff6666; color: white; border: none; padding: 5px 10px; border-radius: 3px; cursor: pointer;">Delete Point</button></p>';
			}
			
			content += '</div>';
			return content;
		}

		// Fetch with timeout to prevent hanging
		function fetchWithTimeout(resource, timeout) {
			return Promise.race([
				fetch(resource),
				new Promise((_, reject) =>
					setTimeout(() => reject(new Error('Request timeout after ' + timeout + 'ms')), timeout)
				)
			]);
		}

		// UI Helper functions
		function showLoading(message = 'Loading...') {
			const loading = document.getElementById('loading');
			loading.textContent = message;
			loading.style.display = 'block';
		}

		function hideLoading() {
			document.getElementById('loading').style.display = 'none';
		}
		
		function showError(message) {
			const error = document.getElementById('error');
			error.textContent = message;
			error.style.display = 'block';
			
			// Auto-hide after 5 seconds
			setTimeout(() => {
				error.style.display = 'none';
			}, 5000);
		}
		
		function hideError() {
			document.getElementById('error').style.display = 'none';
		}
		
		function showMessage(message, duration = 3000) {
			// Create temporary message element
			const messageEl = document.createElement('div');
			messageEl.className = 'loading';
			messageEl.style.backgroundColor = '#e8f5e9';
			messageEl.style.color = '#2e7d32';
			messageEl.textContent = message;
			
			document.body.appendChild(messageEl);
			
			setTimeout(() => {
				messageEl.style.opacity = '0';
				messageEl.style.transition = 'opacity 0.5s';
				
				setTimeout(() => {
					document.body.removeChild(messageEl);
				}, 500);
			}, duration);
		}
	</script>
</body>
</html>`)
}

func handlePositions(w http.ResponseWriter, r *http.Request) {
	positionsMu.RLock()
	defer positionsMu.RUnlock()

	features := make([]map[string]interface{}, 0, len(positions))
	for _, pos := range positions {
		// Skip positions with invalid coordinates (especially important for user-added points)
		if pos.Lat == 0 && pos.Lon == 0 {
			continue
		}

		features = append(features, map[string]interface{}{
			"type": "Feature",
			"geometry": map[string]interface{}{
				"type":        "Point",
				"coordinates": []float64{pos.Lon, pos.Lat},
			},
			"properties": map[string]interface{}{
				"id":          pos.ID,
				"timestamp":   pos.Timestamp.Format(time.RFC3339),
				"accuracy":    pos.AccuracyMm,
				"source":      pos.Source,
				"deviceId":    pos.DeviceID,
				"description": pos.Description,
				"tags":        pos.Tags,
				"isUserAdded": pos.IsUserAdded,
				"lat":         pos.Lat,
				"lng":         pos.Lon,
			},
		})
	}

	w.Header().Set("Content-Type", "application/json")
	if err := json.NewEncoder(w).Encode(map[string]interface{}{
		"type":     "FeatureCollection",
		"features": features,
	}); err != nil {
		log.Printf("Error encoding positions: %v", err)
		http.Error(w, "Internal server error", http.StatusInternalServerError)
	}
}

func handleRoutes(w http.ResponseWriter, r *http.Request) {
	positionsMu.RLock()
	defer positionsMu.RUnlock()

	groups := make(map[string][][]float64)
	for _, pos := range positions {
		// Skip positions with invalid coordinates or without timestamps
		if (pos.Lat == 0 && pos.Lon == 0) || pos.Timestamp.IsZero() {
			continue
		}

		day := pos.Timestamp.Format("2006-01-02")
		groups[day] = append(groups[day], []float64{pos.Lon, pos.Lat})
	}

	w.Header().Set("Content-Type", "application/json")
	if err := json.NewEncoder(w).Encode(groups); err != nil {
		log.Printf("Error encoding routes: %v", err)
		http.Error(w, "Internal server error", http.StatusInternalServerError)
	}
}

func handleAddPoint(w http.ResponseWriter, r *http.Request) {
	// Only accept POST requests
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var point struct {
		Lat         float64  `json:"lat"`
		Lon         float64  `json:"lon"`
		Description string   `json:"description"`
		Tags        []string `json:"tags"`
	}

	if err := json.NewDecoder(r.Body).Decode(&point); err != nil {
		log.Printf("Invalid request data: %v", err)
		http.Error(w, "Invalid request data", http.StatusBadRequest)
		return
	}

	// Validate required fields
	if point.Description == "" {
		http.Error(w, "Description is required", http.StatusBadRequest)
		return
	}

	positionsMu.Lock()
	defer positionsMu.Unlock()

	// Create a new position with a unique ID
	newID := strconv.Itoa(len(positions))

	newPosition := PositionData{
		ID:          newID,
		Lat:         point.Lat,
		Lon:         point.Lon,
		Description: point.Description,
		Tags:        point.Tags,
		Timestamp:   time.Now(),
		IsUserAdded: true,
		// Default values
		DeviceID: "user",
	}

	positions = append(positions, newPosition)

	if err := saveUserEdits(); err != nil {
		log.Printf("Error saving user edits: %v", err)
		http.Error(w, "Failed to save point", http.StatusInternalServerError)
		return
	}

	// Return the new point data
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusCreated)
	json.NewEncoder(w).Encode(map[string]string{
		"id":      newID,
		"message": "Point added successfully",
	})
}

func handleRemovePoint(w http.ResponseWriter, r *http.Request) {
	// Only accept POST requests
	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	id := r.URL.Query().Get("id")
	if id == "" {
		http.Error(w, "Missing ID parameter", http.StatusBadRequest)
		return
	}

	positionsMu.Lock()
	defer positionsMu.Unlock()

	index := -1
	for i, pos := range positions {
		if pos.ID == id {
			index = i
			break
		}
	}

	if index == -1 {
		http.Error(w, "Point not found", http.StatusNotFound)
		return
	}

	// Check if this is a user-added point
	if !positions[index].IsUserAdded {
		http.Error(w, "Cannot delete non-user points", http.StatusForbidden)
		return
	}

	// Remove the position
	positions = append(positions[:index], positions[index+1:]...)

	if err := saveUserEdits(); err != nil {
		log.Printf("Error saving user edits after deletion: %v", err)
		http.Error(w, "Failed to save changes", http.StatusInternalServerError)
		return
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(map[string]string{
		"message": "Point deleted successfully",
	})
}

func saveUserEdits() error {
	var edits []TimelineEdit

	for _, pos := range positions {
		if pos.IsUserAdded {
			edits = append(edits, TimelineEdit{
				DeviceID:    pos.DeviceID,
				Description: pos.Description,
				Tags:        pos.Tags,
				// Include RawSignal for coordinates
				RawSignal: &RawSignal{
					Signal: Signal{
						Position: &Position{
							Point: Point{
								LatE7: int(pos.Lat * 1e7),
								LngE7: int(pos.Lon * 1e7),
							},
							AccuracyMm: pos.AccuracyMm,
							Source:     "user_added",
							Timestamp:  pos.Timestamp.Format(time.RFC3339),
						},
					},
					AdditionalTimestamp: pos.Timestamp.Format(time.RFC3339),
					Metadata: Metadata{
						Platform: "web",
					},
				},
			})
		}
	}

	data, err := json.MarshalIndent(TimelineData{TimelineEdits: edits}, "", "  ")
	if err != nil {
		return fmt.Errorf("error marshaling edits: %w", err)
	}

	if err := os.WriteFile("TimelineNewEdits.json", data, 0644); err != nil {
		return fmt.Errorf("error writing edits: %w", err)
	}

	return nil
}
