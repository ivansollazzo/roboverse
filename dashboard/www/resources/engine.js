/*
    Roboverse - Dashboard Engine

    This file is responsible for managing the file system interactions
    and providing a user-friendly interface for navigating and viewing
    files and folders.

    Written by: Ivan Sollazzo
*/

// Initial variables
let currentPath = '';
let fileStructure = {};
let filteredFiles = null;
const baseDataPath = 'roboverse_data';
let currentFile = null;
let lastRenderedFile = null;
let lastFileContent = null;

// Initialize the application
window.onload = initializeApp;

// Function to initialize the application
function initializeApp() {
    navigateTo('');
    setInterval(autoRefreshDirectory, 10000);
}

// Function to get display name for a file or folder
function getDisplayName(name, item) {
    if (item.type === 'folder') {
        if (name.startsWith('unicycle_')) {
            return name.replace('unicycle_', 'Unicycle ').replace(/_/g, '');
        } else if (name === 'storico') {
            return 'Data History';
        }
    } else if (item.type === 'file') {
        if (name.startsWith('knowledge_')) {
            return name.replace('knowledge_', 'Knowledge ').replace(/_/g, ' ').replace('.csv', '');
        }
    }
    return name;
}

// Function to load file structure
async function loadFileStructure(path) {
    showLoading();
    const fetchPath = path ? `${baseDataPath}/${path}` : baseDataPath;

    try {
        const response = await fetch(`${fetchPath}/`);
        if (!response.ok) {
            throw new Error(`Status ${response.status}. Unable to fetch files in '${fetchPath}'. Please ensure that directory listing is enabled.`);
        }
        const html = await response.text();
        const files = parseDirectoryListing(html, fetchPath);

        fileStructure = buildFileStructure(files);
        renderFileTree();
        hideLoading();

    } catch (error) {
        console.error('Error loading file structure:', error);
        fileStructure = {};
        renderFileTree();
        showError(error.message);
    }
}

// Function to parse directory listing HTML
function parseDirectoryListing(html, currentFullPath) {
    const files = [];
    const parser = new DOMParser();
    const doc = parser.parseFromString(html, 'text/html');
    const links = doc.querySelectorAll('a[href]');

    links.forEach(link => {
        let href = link.getAttribute('href');
        const text = link.textContent.trim();

        if (text === 'Parent Directory' || href.startsWith('?') || href.startsWith('../')) {
            return;
        }

        const isDirectory = href.endsWith('/');
        const name = decodeURIComponent(href.replace(/\/$/, ''));

        if (isDirectory) {
            files.push({ name: name, type: 'folder', path: `${currentFullPath}/${name}` });
        } else if (name.endsWith('.csv')) {
            files.push({ name: name, type: 'file', path: `${currentFullPath}/${name}` });
        }
    });
    return files;
}

// Function to automatically refresh the directory
async function autoRefreshDirectory() {
    console.log('Refreshing directory...');
    const fetchPath = currentPath ? `${baseDataPath}/${currentPath}` : baseDataPath;
    try {
        const response = await fetch(`${fetchPath}/`);
        const html = await response.text();
        const files = parseDirectoryListing(html, fetchPath);

        fileStructure = buildFileStructure(files);
        renderFileTree();

        if (currentFile) {
            const currentFileElements = document.querySelectorAll('.tree-file span:nth-child(2)');
            const newFileElement = Array.from(currentFileElements).find(el => {
                const originalName = Object.keys(fileStructure).find(key => 
                    fileStructure[key].name === currentFile.name || key === currentFile.name
                );
                return originalName && el.textContent === getDisplayName(originalName, fileStructure[originalName]);
            });
            if (newFileElement) {
                lastRenderedFile = newFileElement.closest('.tree-file');
                refreshCurrentFile(currentFile);
            }
        }
    } catch (error) {
        console.error('Auto-refresh failed:', error);
    }
}

// Function to build the file structure
function buildFileStructure(files) {
    const structure = {};
    files.sort((a, b) => {
        if (a.type === b.type) return a.name.localeCompare(b.name);
        return a.type === 'folder' ? -1 : 1;
    }).forEach(file => {
        structure[file.name] = file;
    });
    return structure;
}

// Function to render the file tree
function renderFileTree() {
    const treeElement = document.getElementById('file-tree');
    const itemsToRender = filteredFiles || fileStructure;
    treeElement.innerHTML = '';

    if (Object.keys(itemsToRender).length === 0) {
        treeElement.innerHTML = '<li style="text-align: center; color: #999; padding: 20px;">No files or folders found.</li>';
        return;
    }

    for (const [name, item] of Object.entries(itemsToRender)) {
        const listItem = document.createElement('li');
        listItem.className = 'tree-item';

        const displayName = getDisplayName(name, item);

        if (item.type === 'folder') {
            listItem.innerHTML = `<div class="tree-folder"><span class="folder-icon">🤖</span><span>${displayName}</span></div>`;
            listItem.onclick = () => navigateTo(`${currentPath ? currentPath + '/' : ''}${name}`);
            if (currentFile && currentFile.type === 'folder' && currentFile.name === name) {
                listItem.querySelector('.tree-folder').classList.add('active');
                lastRenderedFile = listItem.querySelector('.tree-folder');
            }
        } else {
            listItem.innerHTML = `<div class="tree-file"><span class="file-icon">🧠</span><span>${displayName}</span></div>`;
            const fileElement = listItem.querySelector('.tree-file');
            fileElement.onclick = (event) => openFile(name, item, event.currentTarget);
            if (currentFile && currentFile.name === name) {
                fileElement.classList.add('active');
                lastRenderedFile = fileElement;
            }
        }
        treeElement.appendChild(listItem);
    }
}

// Function to open file
async function openFile(fileName, fileInfo, element) {
    document.querySelectorAll('.tree-file, .tree-folder').forEach(el => el.classList.remove('active'));
    element.classList.add('active');
    showLoading();
    currentFile = fileInfo;
    lastRenderedFile = element;

    try {
        const response = await fetch(fileInfo.path);
        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);

        const csvContent = await response.text();
        lastFileContent = csvContent;
        const parsedData = parseCSV(csvContent);

        const size = response.headers.get('content-length');
        const lastModified = response.headers.get('last-modified');
        const metadata = {
            size: size ? formatFileSize(parseInt(size)) : 'N/A',
            modified: lastModified ? new Date(lastModified).toLocaleDateString('en-US') : 'N/A'
        };

        displayCSVData(parsedData, fileName, metadata);
    } catch (error) {
        console.error('Error loading CSV file:', error);
        showError('Error loading CSV file: ' + fileName);
        currentFile = null;
        lastRenderedFile = null;
        lastFileContent = null;
    }
}

// Function to refresh the current file
async function refreshCurrentFile(fileInfo) {
    try {
        const response = await fetch(fileInfo.path);
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const csvContent = await response.text();
        
        if (lastFileContent !== csvContent) {
            console.log('File content changed, updating display:', fileInfo.name);
            lastFileContent = csvContent;
            const parsedData = parseCSV(csvContent);

            const size = response.headers.get('content-length');
            const lastModified = response.headers.get('last-modified');
            const metadata = {
                size: size ? formatFileSize(parseInt(size)) : 'N/A',
                modified: lastModified ? new Date(lastModified).toLocaleDateString('en-US') : 'N/A'
            };

            updateCSVDisplay(parsedData, fileInfo.name, metadata);
        }

    } catch (error) {
        console.error('Error refreshing CSV file:', error);
        showError('Error refreshing CSV file: ' + fileInfo.name);
    }
}

// Function to display CSV data
function displayCSVData(csvData, fileName, metadata) {
    document.getElementById('welcome').style.display = 'none';
    const csvViewerElement = document.getElementById('csv-viewer');
    csvViewerElement.style.display = 'block';
    csvViewerElement.classList.add('fade-in');

    updateCSVDisplay(csvData, fileName, metadata);
    hideLoading();
}

// Function to update the CSV display
function updateCSVDisplay(csvData, fileName, metadata) {

    if (fileName.startsWith('knowledge_')) {
        humanFileName = fileName.replace('knowledge_', 'Knowledge ').replace(/_/g, ' ').replace('.csv', '');
        document.getElementById('csv-title').textContent = humanFileName;
    }
    else {
        document.getElementById('csv-title').textContent = fileName;
    }


    const infoText = `${csvData.length} rows, ${csvData[0]?.length || 0} columns`;
    const sizeText = metadata.size !== 'N/A' ? ` • ${metadata.size}` : '';
    const modifiedText = metadata.modified !== 'N/A' ? ` • Modified: ${metadata.modified}` : '';
    document.getElementById('csv-info').textContent = infoText + sizeText + modifiedText;

    const tableElement = document.getElementById('csv-table');
    tableElement.innerHTML = '';
    if (!csvData || csvData.length <= 1) {
        tableElement.innerHTML = '<tr><td class="error">The CSV file is empty or unreadable.</td></tr>';
        return;
    }

    const thead = tableElement.createTHead();
    const headerRow = thead.insertRow();
    csvData[0].forEach(headerText => {
        const th = document.createElement('th');
        if (headerText === 'id') {
            th.textContent = 'Location ID';
        } else if (headerText === 'temperature') {
            th.textContent = 'Temperature (°C)';
        } else if (headerText === 'humidity') {
            th.textContent = 'Humidity (%)';
        } else if (headerText === 'air_quality') {
            th.textContent = 'Air Quality (%)';
        } else {
            th.textContent = headerText;
        }
        headerRow.appendChild(th);
    });

    const tbody = tableElement.createTBody();
    for (let i = 1; i < csvData.length; i++) {
        const row = tbody.insertRow();
        csvData[i].forEach(cellText => {
            const cell = row.insertCell();
            cell.textContent = cellText;
        });
    }
}

// Function to navigate to a different directory
function navigateTo(path) {
    currentPath = path;
    document.getElementById('welcome').style.display = 'block';
    document.getElementById('csv-viewer').style.display = 'none';
    updateBreadcrumb();
    loadFileStructure(path);
    currentFile = null;
    lastRenderedFile = null;
    lastFileContent = null;
}

// Function to update breadcrumb navigation
function updateBreadcrumb() {
    const container = document.getElementById('breadcrumb-container');
    container.innerHTML = `<span class="breadcrumb-item" onclick="navigateTo('')">🏠 Roboverse Dataset</span>`;
    const parts = currentPath.split('/').filter(p => p);
    let pathSoFar = '';
    parts.forEach(part => {
        pathSoFar += (pathSoFar ? '/' : '') + part;
        const pathForClick = pathSoFar;

        let displayName = part;


        if (part.startsWith('unicycle_')) {
            displayName = part.replace('unicycle_', 'Unicycle ').replace(/_/g, ' ').replace('.csv', '');
        }

        container.innerHTML += ` <span style="color: #ccc;">›</span> <span class="breadcrumb-item" onclick="navigateTo('${pathForClick}')">${displayName}</span>`;
    });
}

// Function to format file size
function formatFileSize(bytes, decimalPoint) {
    if (bytes === 0) return '0 Bytes';
    const k = 1024;
    const dm = decimalPoint || 2;
    const sizes = ['Bytes', 'KB', 'MB', 'GB', 'TB', 'PB', 'EB', 'ZB', 'YB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(dm)) + ' ' + sizes[i];
}

// Function to show loading
function showLoading() {
    document.getElementById('nojsloaded').style.display = 'none';
    document.getElementById('csv-viewer').style.display = 'none';
    document.getElementById('error').style.display = 'none';
    document.getElementById('loading').style.display = 'block';
}

// Function to hide loading
function hideLoading() {
    document.getElementById('loading').style.display = 'none';
}

// Function to show error
function showError(message) {
    document.getElementById('welcome').style.display = 'none';
    document.getElementById('csv-viewer').style.display = 'none';
    document.getElementById('loading').style.display = 'none';
    const errorElement = document.getElementById('error');
    errorElement.querySelector('p').textContent = `❌ ${message}`;
    errorElement.style.display = 'block';
}

// Function to parse a CSV file
function parseCSV(csvContent) {
    const lines = csvContent
        .split(/\r?\n/)
        .filter(line => line.trim());

    return lines.map(line => {
        const row = [];
        let current = '';
        let inQuotes = false;

        for (let i = 0; i < line.length; i++) {
            const char = line[i];

            if (char === '"') {
                if (inQuotes && line[i + 1] === '"') {
                    current += '"';
                    i++;
                } else {
                    inQuotes = !inQuotes;
                }
            } else if (char === ',' && !inQuotes) {
                row.push(current.trim());
                current = '';
            } else {
                current += char;
            }
        }
        row.push(current.trim());
        return row;
    });
}