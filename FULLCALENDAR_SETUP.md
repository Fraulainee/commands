# 🚀 Quick Setup Guide - FullCalendar

## Step 1: Install Packages

Run this command in your terminal:

```bash
npm install @fullcalendar/react @fullcalendar/core @fullcalendar/daygrid @fullcalendar/timegrid @fullcalendar/interaction
```

Wait for installation to complete...

---

## Step 2: Replace Your Calendar Files

### Option A: Copy Files (Recommended)
1. Save your current `Calendar.jsx` as backup (optional)
2. Replace with `Calendar-FullCalendar.jsx` → rename to `Calendar.jsx`
3. Replace with `Calendar-FullCalendar.css` → rename to `Calendar.css`

### Option B: Update Manually
Open your `Calendar.jsx` and replace all content with the content from `Calendar-FullCalendar.jsx`

---

## Step 3: Verify Your Files

Your `src/` folder should have:
```
src/
├── Calendar.jsx     ← Updated with FullCalendar
├── Calendar.css     ← Styled with thistle/mocha theme
├── Dashboard.jsx
├── App.jsx
└── ... other files
```

---

## Step 4: Run & Test!

```bash
npm run dev
```

Navigate to `/calendar` and you should see:
- ✅ Beautiful calendar with thistle/mocha colors
- ✅ Month/Week/Day view buttons
- ✅ Sample events displayed
- ✅ Click dates to add events
- ✅ Click events to delete them

---

## 🎨 What You Get:

### Features:
- 📅 **Month, Week, Day views**
- ➕ **Click dates to add events**
- 🗑️ **Click events to delete**
- 🖱️ **Drag events to move them**
- 🔄 **Resize events (in week/day view)**
- 📊 **Event statistics**
- 💡 **Helpful tips section**

### Styling:
- 🎨 Thistle & Mocha color scheme
- ✨ Smooth animations
- 📱 Fully responsive
- 🎯 Today's date highlighted
- 💼 Business hours shown
- ⏰ Current time indicator

---

## 🎯 Try These Actions:

1. **Add an event:**
   - Click on any date
   - Enter a title in the prompt
   - See it appear on the calendar!

2. **Delete an event:**
   - Click on an existing event
   - Confirm deletion

3. **Switch views:**
   - Click "month", "week", or "day" buttons
   - See different calendar layouts

4. **Move an event:**
   - Click and drag an event to a new date

5. **Create multi-day event:**
   - Click and drag across multiple dates
   - Enter a title

---

## 🔧 Customization Ideas:

### Change Event Colors:
In `Calendar.jsx`, modify the event objects:
```jsx
{
  id: '1',
  title: 'My Event',
  start: '2025-02-10',
  backgroundColor: '#YOUR_COLOR',  // Change this!
  borderColor: '#YOUR_COLOR'       // And this!
}
```

### Change Calendar Colors:
In `Calendar.css`, modify CSS variables:
```css
:root {
  --fc-event-bg-color: #D8BFD8;      /* Event background */
  --fc-button-bg-color: #D8BFD8;     /* Button color */
  --fc-today-bg-color: rgba(216, 191, 216, 0.15);  /* Today highlight */
}
```

### Add More Sample Events:
In `Calendar.jsx`, add to the events array:
```jsx
const [events, setEvents] = useState([
  // ... existing events
  {
    id: '4',
    title: 'Your New Event',
    start: '2025-02-25T10:00:00',
    backgroundColor: '#D8BFD8',
    borderColor: '#C8A8C8'
  }
]);
```

---

## 🎓 Learning Points:

### React Concepts Used:
- ✅ **useState** - Managing events array
- ✅ **Event Handlers** - Click, drag, resize
- ✅ **Props** - Passing data to FullCalendar
- ✅ **Array Methods** - filter, map for event manipulation

### CSS Concepts Used:
- ✅ **CSS Variables** - FullCalendar theming
- ✅ **Deep Selectors** - Styling library components
- ✅ **Gradients** - Beautiful color transitions
- ✅ **Transitions** - Smooth hover effects
- ✅ **Grid Layout** - Stats section
- ✅ **Media Queries** - Responsive design

---

## 📚 FullCalendar Documentation

For more advanced features, check out:
- [FullCalendar Docs](https://fullcalendar.io/docs)
- [Event Object](https://fullcalendar.io/docs/event-object)
- [View API](https://fullcalendar.io/docs/view-api)

---

## 🐛 Troubleshooting

**Issue: Calendar not showing?**
```bash
# Make sure packages are installed
npm list @fullcalendar/react

# If not found, reinstall
npm install @fullcalendar/react @fullcalendar/core @fullcalendar/daygrid @fullcalendar/timegrid @fullcalendar/interaction
```

**Issue: Styles look weird?**
- Make sure you imported `Calendar.css` in your component
- Check that CSS variables are defined in your global CSS

**Issue: Events not appearing?**
- Check browser console for errors (F12)
- Make sure event dates are in correct format: 'YYYY-MM-DD'

---

## 🚀 Next Steps

Now that you have a working calendar, you can:

1. **Add persistent storage** - Save events to localStorage or backend
2. **Create event modal** - Fancy form instead of prompt
3. **Add event categories** - Color-code by type
4. **Integrate with backend** - API for saving/loading events
5. **Add reminders** - Notifications for upcoming events

**Which feature would you like to build next?** 🎯
