# 📅 FullCalendar Setup & Styling Tutorial

## Step 1: Install FullCalendar Packages

FullCalendar requires multiple packages. Install them all:

```bash
npm install @fullcalendar/react @fullcalendar/core @fullcalendar/daygrid @fullcalendar/timegrid @fullcalendar/interaction
```

**What each package does:**
- `@fullcalendar/react` - React wrapper for FullCalendar
- `@fullcalendar/core` - Core FullCalendar functionality
- `@fullcalendar/daygrid` - Month view plugin
- `@fullcalendar/timegrid` - Week/Day view plugins
- `@fullcalendar/interaction` - Click/drag events (for adding events)

---

## Step 2: Understanding Your Calendar Component

Let's break down what each part does:

```jsx
import FullCalendar from '@fullcalendar/react'
import dayGridPlugin from '@fullcalendar/daygrid'    // Month view
import timeGridPlugin from '@fullcalendar/timegrid'  // Week/Day views

export default function Calendar() {
  return (
    <FullCalendar
      // PLUGINS: Enable different view types
      plugins={[ dayGridPlugin, timeGridPlugin ]}
      
      // INITIAL VIEW: What view to show on load
      initialView="dayGridMonth"  // Starts in month view
      
      // HEADER TOOLBAR: Navigation buttons and controls
      headerToolbar={{
        left: 'prev,next today',        // Navigation buttons
        center: 'title',                // Month/Year title
        end: 'dayGridMonth,timeGridWeek,timeGridDay'  // View switcher
      }}
    />
  )
}
```

---

## Step 3: Features to Add

### Add Event Clicking
To make dates clickable and add events:

```jsx
import { useState } from 'react'
import FullCalendar from '@fullcalendar/react'
import dayGridPlugin from '@fullcalendar/daygrid'
import timeGridPlugin from '@fullcalendar/timegrid'
import interactionPlugin from '@fullcalendar/interaction'  // Add this!

export default function Calendar() {
  const [events, setEvents] = useState([
    // Sample events
    {
      id: '1',
      title: 'Team Meeting',
      start: '2025-02-10T10:00:00',
      end: '2025-02-10T11:00:00',
      backgroundColor: '#D8BFD8',  // Thistle color!
      borderColor: '#C8A8C8'
    },
    {
      id: '2',
      title: 'Project Deadline',
      start: '2025-02-15',
      backgroundColor: '#C8A8C8',
      borderColor: '#B8A0B8'
    }
  ])

  // Handle date clicks
  const handleDateClick = (info) => {
    const title = prompt('Enter event title:')
    if (title) {
      const newEvent = {
        id: String(Date.now()),
        title: title,
        start: info.dateStr,
        backgroundColor: '#D8BFD8',
        borderColor: '#C8A8C8'
      }
      setEvents([...events, newEvent])
    }
  }

  // Handle event clicks
  const handleEventClick = (info) => {
    if (window.confirm(`Delete event '${info.event.title}'?`)) {
      setEvents(events.filter(e => e.id !== info.event.id))
    }
  }

  return (
    <FullCalendar
      plugins={[ dayGridPlugin, timeGridPlugin, interactionPlugin ]}
      initialView="dayGridMonth"
      headerToolbar={{
        left: 'prev,next today',
        center: 'title',
        end: 'dayGridMonth,timeGridWeek,timeGridDay'
      }}
      events={events}           // Pass your events
      dateClick={handleDateClick}   // Handle date clicks
      eventClick={handleEventClick} // Handle event clicks
      editable={true}           // Allow drag/drop
      selectable={true}         // Allow date range selection
    />
  )
}
```

---

## Step 4: FullCalendar Options Explained

Here are useful options you can add:

```jsx
<FullCalendar
  // VIEW OPTIONS
  initialView="dayGridMonth"
  
  // HEADER
  headerToolbar={{
    left: 'prev,next today',
    center: 'title',
    end: 'dayGridMonth,timeGridWeek,timeGridDay'
  }}
  
  // TIME OPTIONS
  slotMinTime="06:00:00"   // Start time in week/day view
  slotMaxTime="22:00:00"   // End time in week/day view
  allDaySlot={true}        // Show "all day" row
  
  // INTERACTION
  editable={true}          // Can drag/resize events
  selectable={true}        // Can select date ranges
  selectMirror={true}      // Visual feedback when selecting
  dayMaxEvents={true}      // Show "more" link when too many events
  
  // EVENTS
  events={events}
  
  // EVENT HANDLERS
  dateClick={handleDateClick}
  eventClick={handleEventClick}
  select={handleDateSelect}  // When range is selected
  eventDrop={handleEventDrop}  // When event is dragged
  
  // STYLING
  height="auto"            // Auto height
  aspectRatio={1.8}        // Width-to-height ratio
  
  // WEEK OPTIONS
  weekends={true}          // Show weekends
  firstDay={0}            // 0=Sunday, 1=Monday
  
  // BUSINESS HOURS
  businessHours={{
    daysOfWeek: [1, 2, 3, 4, 5],  // Monday-Friday
    startTime: '09:00',
    endTime: '17:00'
  }}
/>
```

---

## Step 5: Custom CSS Variables

FullCalendar uses CSS variables that you can override!

**Common CSS Variables:**
```css
:root {
  /* Calendar background */
  --fc-border-color: #E6D7E6;
  --fc-bg-event-color: #D8BFD8;
  --fc-bg-event-opacity: 0.9;
  
  /* Text colors */
  --fc-event-text-color: #2B1F1F;
  --fc-neutral-text-color: #2B1F1F;
  
  /* Today's date highlight */
  --fc-today-bg-color: rgba(216, 191, 216, 0.15);
  
  /* Button colors */
  --fc-button-bg-color: #D8BFD8;
  --fc-button-border-color: #C8A8C8;
  --fc-button-hover-bg-color: #C8A8C8;
  --fc-button-active-bg-color: #B8A0B8;
}
```

---

## Next Steps

1. ✅ Install packages
2. ✅ Update your Calendar.jsx with features
3. ✅ Add custom CSS styling (next file!)
4. ✅ Test clicking dates to add events
5. ✅ Test switching between views

Ready for the custom CSS? Let's make it beautiful! 🎨
